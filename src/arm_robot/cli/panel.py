#!/usr/bin/env python3
"""ZDT 六轴机械臂 CAN 直连安全调试面板 (curses TUI).

⚠⚠ 安全须知 (务必先读) ⚠⚠
  · 本工具**绕过 STM32 网关直接控制电机** — 固件限位开关保护不可用,
    只有本面板的软限位/限速/看门狗 + 驱动器堵转保护作为安全包络.
  · 仅限 bring-up/调试, 禁止接入正常 VLA/LeRobot 推理路径.
  · 电机默认保持力矩 (不断电); 禁用重力关节 (J2/J3) 前确认手臂有支撑.
  · 任意时刻按 X / Esc 广播急停 (0xFE, 停止运动但保持力矩).

用法:
  python zdt_panel.py [--iface can0] [--addr-scheme auto|firmware|pc]
                      [--watchdog 3.0] [--dump]
  --dump: 一次性文本模式 (枚举 + 状态表), 适合脚本/CI.

键位: e 枚举 | j/k/↑↓ 选择 | 1-6 跳关节 | Enter 选中 | a 臂置(J2/J3需确认) |
      E/D 全使能/全禁用 | x 停选中 | X/Esc 急停 | ./ , 微步进 ± |
      + / - 调步进幅值 | s 限速 | t/T 单次/连续遥测 | p/m 读/改参数 |
      i 改ID(不可逆) | r 复位急停闩锁 | ? 帮助 | q 退出
"""
import argparse
import logging
import sys
import time
from typing import Optional

import curses

from arm_robot.driver.frames import add_checksum
from arm_robot.controller.params import (
    CAN_BAUD_LABELS, RESPONSE_LABELS, encode_48_write, encode_ae_id_change,
    parse_42_response,
)
from arm_robot.controller.safety import (
    JOINTS, SafetyError, SafetyMachine,
)
from arm_robot.driver.scan import scan_bus
from arm_robot.driver.zdt_bus import ZdtBus

logging.basicConfig(level=logging.WARNING)

ACC_DEFAULT: int = 20
ENABLE_BODY: bytes = bytes([0xF3, 0xAB, 0x01, 0x00])
DISABLE_BODY: bytes = bytes([0xF3, 0xAB, 0x00, 0x00])
F_READ_FLAG: int = 0x3A
F_READ_CUR: int = 0x27
F_READ_PARAMS: int = 0x42
F_STOP: int = 0xFE
AUTO_TELEMETRY_EVERY: int = 5      # 主循环每 5 轮自动遥测一次 (~1Hz)
STEP_POLL_TIMEOUT_S: float = 2.0   # 步进等待到位/堵转的超时
STEP_FLAG_TIMEOUT_S: float = 0.1   # 步进轮询 0x3A 的单次请求超时


def make_fd_payload(dir_byte: int, speed_rpm: float, pulses: int,
                    acc: int = ACC_DEFAULT) -> bytes:
    """组装 0xFD 位置命令负载 (固件 Emm_V5_Pos_Control 兼容, 已真机验证).

    2026-08-23: 0xFD 速度字段 = RPM 直传 (手册 0x05DC=1500RPM), 修复旧 ×10 bug.
    """
    vel = int(max(1, round(abs(speed_rpm)))) & 0xFFFF
    return add_checksum(bytes([
        0xFD, dir_byte, (vel >> 8) & 0xFF, vel & 0xFF, acc,
        (pulses >> 24) & 0xFF, (pulses >> 16) & 0xFF,
        (pulses >> 8) & 0xFF, pulses & 0xFF, 0x00, 0x00]))


class PanelApp:
    """curses 面板主循环. 安全状态全部走 SafetyMachine, UI 仅作薄封装."""

    def __init__(self, stdscr, bus: ZdtBus, sm: SafetyMachine,
                 args, latch: dict):
        self.stdscr = stdscr
        self.bus = bus
        self.sm = sm
        self.args = args
        self.latch = latch
        self.scheme = None
        self.cursor = 0
        self.status = "按 ? 查看帮助; 先 e 枚举"
        self.help_mode = False
        self.continuous_telemetry = False
        self.any_enabled = False
        self.quit = False
        self._tick = 0
        self.prompt = None
        self.prompt_buf = ""
        self.confirm = None               # 内联确认: (消息, 回调); y=确认 n/Esc=取消
        self.pending_step = None          # 非阻塞步进: (can_id, MovePlan, deadline)

    # ── 主循环 ──
    def loop(self) -> None:
        self.stdscr.nodelay(True)
        curses.curs_set(0)
        while not self.quit:
            if self.latch.get("estop"):
                self.sm.e_stop()
                self.latch["estop"] = False
                self.any_enabled = False
                self.status = "⚠ 看门狗/外部 e_stop 已触发 (STOPPED 闩锁)"
            self._tick += 1
            key = self.stdscr.getch()
            # 🔴 全局急停最高优先级: 任何状态 (含输入框/步进中) 都先响应 X
            if key in (ord('X'),):
                self.prompt = None
                self.confirm = None
                self.stdscr.nodelay(True)
                self.do_estop()
                continue
            # 非阻塞步进轮询 + 自动遥测 (每轮推进, 即使无按键)
            if self.pending_step is not None:
                self._poll_step()
            if self.continuous_telemetry and self.sm.selected_id:
                self._telemetry_once(self.sm.selected_id)
            elif self._tick % AUTO_TELEMETRY_EVERY == 0:
                self._auto_telemetry()
            self.draw()
            if key == -1:
                time.sleep(0.05)
                continue
            if self.confirm is not None:
                self._handle_confirm_key(key)
                continue
            if self.prompt is not None:
                self._handle_prompt_key(key)
                continue
            self._handle_key(key)

    # ── 渲染 ──
    def draw(self) -> None:
        self.stdscr.erase()
        h, w = self.stdscr.getmaxyx()
        lines = []
        scheme = self.scheme or getattr(self.args, "addr_scheme", "auto")
        lines.append((f"ZDT 6-DOF CAN 直连调试面板   状态:{self.sm.phase.name:<12}"
                      f"scheme:{scheme}", curses.A_BOLD))
        if self.bus.watchdog_triggered:
            lines.append(("⚠⚠ 看门狗已触发 — 需要 r 复位", curses.A_REVERSE))
        lines.append(("J# 名称             ID   FW/HW   使能 到位 堵转 电流mA MStep 减速比 备注",
                      curses.A_UNDERLINE))
        motors = sorted(self.sm.motors.values(),
                        key=lambda m: (m.joint_slot is None, m.joint_slot or 0, m.can_id))
        for m in motors:
            jm = JOINTS[m.joint_slot] if m.joint_slot is not None else None
            slot = f"J{m.joint_slot+1}" if jm else "?"
            name = jm.name if jm else "?"
            fw = f"{m.fw_ver[0]:#x}/{m.fw_ver[1]:#x}" if m.fw_ver else "-"
            en = "Y" if m.flags & 0x01 else "-"
            arr = "Y" if m.flags & 0x02 else "-"
            stl = "!" if m.flags & 0x04 else "-"
            cur = f"{m.current_ma:.0f}" if m.current_ma is not None else "-"
            mstep = str(m.mstep) if m.mstep else "-"
            ratio = f"{m.ratio:.2f}" if m.ratio else "-"
            sel = "▶" if m.selected else " "
            gravity = "⚙" if (jm and jm.gravity) else " "
            row = (f"{sel}{slot:<3}{name:<14}0x{m.can_id:02X}  {fw:<8}"
                   f"{en}   {arr}   {stl}    {cur:<6}{mstep:<5}{ratio:<6}{gravity}{m.note}")
            lines.append((row, curses.A_REVERSE if m.selected else curses.A_NORMAL))
        lines.append(("", 0))
        if self.sm.selected_id and self.sm.selected_id in self.sm.motors:
            m = self.sm.motors[self.sm.selected_id]
            lines.append((f"选中 0x{m.can_id:02X}  跟踪角 {m.tracked_deg:.1f}°  "
                          f"步进 {self.sm.step_size_deg:.1f}°  限速 {self.sm.speed_cap_rpm:.0f}RPM", 0))
            if m.params is not None:
                p = m.params
                lines.append((f"参数: MStep={p.mstep} CAN_Baud={CAN_BAUD_LABELS.get(p.can_baud,'?')} "
                              f"Checksum={p.checksum} Response={RESPONSE_LABELS.get(p.response,'?')} "
                              f"S_PosTDP={p.pos_tdp} Ma_Limit={p.ma_limit_ma}mA "
                              f"Vm_Limit={p.vm_limit_rpm}RPM Clog_Pro={p.clog_pro}", 0))
                if p.warning:
                    lines.append((f"⚠ {p.warning}", 0))
                if not p.mstep_is_default:
                    lines.append(("⚠ MStep≠16 → 3200脉冲/圈假设失配, 需复核标定!", curses.A_REVERSE))
        if self.pending_step is not None:
            cid = self.pending_step[0]
            for m in motors:
                if m.can_id == cid:
                    lines.append((f"▶ 运动中 J{m.joint_slot+1 if m.joint_slot is not None else '?'} "
                                  f"(按 X 可急停)", curses.A_REVERSE))
        if self.confirm is not None:
            lines.append((f">> {self.confirm[0]} [y/N]  (X=急停)", curses.A_REVERSE))
        elif self.prompt is not None:
            lines.append((f">> {self.prompt[0]} [{self.prompt_buf}]", curses.A_REVERSE))
        elif self.status:
            lines.append((self.status, 0))
        else:
            lines.append(("e枚举 a臂置 E全使能 D全禁用 X急停 . ,步进 t遥测 p参数 ?帮助 q退出", 0))
        if self.help_mode:
            for ln in HELP_TEXT.splitlines():
                lines.append((ln, 0))
        for i, (txt, attr) in enumerate(lines):
            if i >= h - 1:
                break
            try:
                self.stdscr.addnstr(i, 0, txt[:w - 1], w - 1, attr)
            except curses.error:
                pass
        self.stdscr.refresh()

    # ── 操作 ──
    def do_scan(self) -> None:
        self.status = "枚举中..."
        self.stdscr.refresh()
        result = scan_bus(self.bus, id_range=(1, self.args.scan_max),
                          forced_scheme=None
                          if self.args.addr_scheme == "auto"
                          else self.args.addr_scheme)
        self.sm.set_scan(result.found)
        self.scheme = result.scheme
        self.cursor = 0
        if not result.found:
            self.status = "; ".join(result.warnings) or "无电机"
        else:
            self.status = ("scheme=%s 发现 %d 电机: %s"
                           % (result.scheme or "?",
                              len(result.found),
                              ", ".join(f"0x{c:02X}" for c in sorted(result.found))))
            if result.warnings:
                self.status += " | " + "; ".join(result.warnings[:3])

    def _select(self, motor) -> None:
        try:
            self.sm.select(motor.can_id)
        except SafetyError as e:
            self.status = str(e)

    def do_arm(self) -> None:
        if self.sm.selected_id is None:
            self.status = "先选择电机"
            return
        m = self.sm.motors[self.sm.selected_id]
        jm = JOINTS[m.joint_slot] if m.joint_slot is not None else None
        if jm and jm.gravity:
            self._ask_confirm(f"⚠ {jm.name}(J{jm.index+1}) 重力关节, 臂置?",
                              lambda ok: self._arm_confirm(ok, m.can_id))
        else:
            self._arm_confirm(True, m.can_id)

    def _arm_confirm(self, confirmed, can_id) -> None:
        try:
            self.sm.arm(can_id=can_id, gravity_confirmed=confirmed)
            self.bus.send_payload(can_id, add_checksum(ENABLE_BODY))
            self.any_enabled = True
            self.bus.set_watchdog_armed(True)
            self.status = f"0x{can_id:02X} 已臂置 (使能保持)"
        except SafetyError as e:
            self.status = str(e)

    def do_disarm(self) -> None:
        self.sm.disarm()
        self.status = "已失臂置 (电机仍保持力矩)"

    def do_enable_all(self) -> None:
        self._prompt("全使能 (所有电机保持力矩)? 输入 CONFIRM", self._enable_all_confirm)

    def _enable_all_confirm(self, s) -> None:
        if s != "CONFIRM":
            self.status = "取消"
            return
        for cid in self.sm.motors:
            self.bus.send_payload(cid, add_checksum(ENABLE_BODY))
        self.any_enabled = True
        self.bus.set_watchdog_armed(True)
        self.status = "全使能"

    def do_disable_all(self) -> None:
        self._prompt("全禁用 (⚠ 重力关节 J2/J3 会失去力矩! 需支撑) 输入 CONFIRM",
                     self._disable_all_confirm)

    def _disable_all_confirm(self, s) -> None:
        if s != "CONFIRM":
            self.status = "取消"
            return
        for cid in self.sm.motors:
            self.bus.send_payload(cid, add_checksum(DISABLE_BODY))
        self.any_enabled = False
        self.bus.set_watchdog_armed(False)
        self.status = "全禁用 (力矩关闭)"

    def do_estop(self) -> None:
        self.bus.stop_all()
        self.sm.e_stop()
        self.any_enabled = False
        self.status = "⚠ 已广播 e_stop (停止运动, 保持力矩) — 按 r 复位"

    def do_re_arm(self) -> None:
        self._ask_confirm("复位急停闩锁?", lambda ok: self._rearm_confirm(ok))

    def _rearm_confirm(self, confirmed) -> None:
        try:
            self.sm.re_arm(confirmed=confirmed)
            self.status = "已复位, 可重新枚举/臂置"
        except SafetyError as e:
            self.status = str(e)

    def do_step(self, sign: int) -> None:
        """发起微步进 (非阻塞): 全部门禁后发送, 主循环每轮轮询到位/堵转.

        运动期间 X/Esc 仍然秒响应 (主循环未被阻塞).
        """
        if self.pending_step is not None:
            self.status = "已有步进在途, 等待完成"
            return
        try:
            plan = self.sm.request_step(sign * self.sm.step_size_deg)
        except SafetyError as e:
            self.status = str(e)
            return
        m = self.sm.motors[self.sm.selected_id]
        payload = make_fd_payload(plan.dir_byte, plan.speed_rpm, plan.pulses)
        self.bus.send_payload(m.can_id, payload)
        self.pending_step = (m.can_id, plan, time.monotonic() + STEP_POLL_TIMEOUT_S)
        self.status = (f"STEPPING {plan.joint_name} {plan.delta_deg:+.2f}° "
                       f"dir={plan.dir_byte} pulses={plan.pulses} @{plan.speed_rpm:.0f}RPM")

    def _poll_step(self) -> None:
        """每轮推进非阻塞步进: 查 0x3A 到位/堵转/超时."""
        cid, plan, deadline = self.pending_step
        m = self.sm.motors.get(cid)
        if m is None:
            self._finish_step("电机丢失")
            return
        fd = self.bus.request(cid, add_checksum(bytes([F_READ_FLAG])),
                              F_READ_FLAG, timeout_s=STEP_FLAG_TIMEOUT_S)
        if fd and len(fd) > 1:
            m.flags = fd[1]
            if m.flags & 0x04:
                self._finish_step("堵转!")
                return
            if m.flags & 0x02:
                self._finish_step("到位")
                return
        elif time.monotonic() > deadline:
            self._finish_step("超时")
            return
        self.status = (f"STEPPING {plan.joint_name} {plan.delta_deg:+.2f}° "
                       f"flag=0x{m.flags:02X}...")

    def _finish_step(self, result: str) -> None:
        cid, plan, _ = self.pending_step
        self.pending_step = None
        m = self.sm.motors.get(cid)
        if m is not None:
            m.tracked_deg = plan.target_deg
        self.sm.step_complete()
        self.status = f"STEP {plan.joint_name} {plan.delta_deg:+.2f}° → {plan.target_deg:.1f}° [{result}]"

    def _telemetry_once(self, cid: int) -> None:
        m = self.sm.motors.get(cid)
        if not m:
            return
        fd = self.bus.request(cid, add_checksum(bytes([F_READ_FLAG])),
                              F_READ_FLAG, timeout_s=0.15)
        if fd and len(fd) > 1:
            m.flags = fd[1]
        cd = self.bus.request(cid, add_checksum(bytes([F_READ_CUR])),
                              F_READ_CUR, timeout_s=0.15)
        if cd and len(cd) > 1:
            m.current_ma = float((cd[1] << 8) | cd[2])
        if self._tick % 4 == 0:
            self.status = f"遥测 0x{cid:02X}: flag=0x{m.flags:02X} cur={m.current_ma:.0f}mA"

    def _auto_telemetry(self) -> None:
        """低频自动刷所有在线电机 flag + 选中电机电流 (上位机感)."""
        for cid, m in list(self.sm.motors.items()):
            if not m.online:
                continue
            fd = self.bus.request(cid, add_checksum(bytes([F_READ_FLAG])),
                                  F_READ_FLAG, timeout_s=0.08)
            if fd and len(fd) > 1:
                m.flags = fd[1]
            if self.sm.selected_id == cid:
                cd = self.bus.request(cid, add_checksum(bytes([F_READ_CUR])),
                                      F_READ_CUR, timeout_s=0.08)
                if cd and len(cd) > 1:
                    m.current_ma = float((cd[1] << 8) | cd[2])

    # ── 内联确认 [y/N] ──
    def _ask_confirm(self, msg: str, callback) -> None:
        self.confirm = (msg, callback)
        self.stdscr.nodelay(True)

    def _handle_confirm_key(self, key: int) -> None:
        msg, cb = self.confirm
        if key in (ord('y'), ord('Y')):
            self.confirm = None
            cb(True)
        elif key in (ord('n'), ord('N'), 27):
            self.confirm = None
            self.status = "取消"
            # 取消不调用回调 (动作不执行)

    def do_params_read(self) -> None:
        cid = self.sm.selected_id
        if cid is None:
            self.status = "先选择电机"
            return
        data = self.bus.request_multi(cid, add_checksum(bytes([F_READ_PARAMS, 0x6C])),
                                      F_READ_PARAMS, timeout_s=0.5)
        if data is None:
            self.status = "参数读取超时 (0x42 无响应)"
            return
        try:
            m = self.sm.motors[cid]
            m.params = parse_42_response(data)
            m.mstep = m.params.mstep
            self.status = "参数已读取"
        except ValueError as e:
            self.status = f"参数解析失败: {e}"

    def do_params_write(self) -> None:
        if self.sm.selected_id is None:
            self.status = "先选择电机"
            return
        self._prompt("改参数: <字段号> <值> [2字节字段加 w]  (如 18 2 或 12 1800 w)",
                     self._params_write_go)

    def _params_write_go(self, s) -> None:
        m = self.sm.motors.get(self.sm.selected_id)
        if not m or m.params is None:
            self.status = "先 p 读取参数"
            return
        try:
            parts = s.split()
            field = int(parts[0])
            value = int(parts[1])
            width = 2 if len(parts) > 2 and parts[2] == "w" else 1
            new_block = m.params.patched(field, value, width)
            payload = encode_48_write(new_block, save=True)
            self.bus.send_payload(self.sm.selected_id, payload)
            m.params = new_block
            self.status = f"已写参数 #{field} = {value} (请重读验证)"
        except (ValueError, KeyError, IndexError) as e:
            self.status = f"参数写失败: {e}"

    def do_id_change(self) -> None:
        if self.sm.selected_id is None:
            self.status = "先选择电机"
            return
        self._prompt("改 ID 不可逆: 输入 '<新ID> CONFIRM' (如 16 CONFIRM)",
                     self._id_change_go)

    def _id_change_go(self, s) -> None:
        try:
            parts = s.split()
            new_id = int(parts[0], 0)
            if len(parts) < 2 or parts[1] != "CONFIRM":
                self.status = "需 CONFIRM 确认"
                return
            if new_id in self.sm.motors and new_id != self.sm.selected_id:
                self.status = f"新 ID 0x{new_id:02X} 与在线电机冲突"
                return
            payload = encode_ae_id_change(new_id)
            self.bus.send_payload(self.sm.selected_id, payload)
            self.status = f"已发送改 ID → 0x{new_id:02X}, 重扫验证 (旧 ID 永久失效)"
        except ValueError as e:
            self.status = f"改 ID 失败: {e}"

    def do_set_speed(self) -> None:
        self._prompt("限速 (RPM, ≤30):", self._set_speed_go)

    def _set_speed_go(self, s) -> None:
        try:
            v = float(s)
            cap = min(max(1.0, v), 30.0)
            self.sm.speed_cap_rpm = cap
            self.status = f"限速 {cap:.0f} RPM"
        except ValueError:
            self.status = "非法速度"

    def do_set_step(self) -> None:
        self._prompt("步进幅值 (0.1-5°):", self._set_step_go)

    def _set_step_go(self, s) -> None:
        try:
            v = float(s)
            step = min(max(0.1, v), 5.0)
            self.sm.step_size_deg = step
            self.status = f"步进幅值 {step:.1f}°"
        except ValueError:
            self.status = "非法步进"

    # ── 输入提示 ──
    def _prompt(self, msg: str, callback) -> None:
        self.prompt = (msg, callback)
        self.prompt_buf = ""
        self.stdscr.nodelay(False)

    def _handle_prompt_key(self, key: int) -> None:
        if key in (10, 13):
            cb = self.prompt[1]
            self.prompt = None
            self.stdscr.nodelay(True)
            cb(self.prompt_buf)
        elif key == 27:
            self.prompt = None
            self.stdscr.nodelay(True)
            self.status = "取消"
        elif key in (8, 127, curses.KEY_BACKSPACE):
            self.prompt_buf = self.prompt_buf[:-1]
        elif 32 <= key < 127:
            self.prompt_buf += chr(key)

    # ── 键分发 ──
    def _handle_key(self, key: int) -> None:
        motors = sorted(self.sm.motors.values(),
                        key=lambda m: (m.joint_slot is None, m.joint_slot or 0, m.can_id))
        if key == ord('?'):
            self.help_mode = not self.help_mode
            return
        if key == ord('q'):
            if self.any_enabled:
                self._prompt("有电机使能中, 输入 QUIT 退出 (不断电不解力矩)",
                             self._quit_confirm)
            else:
                self.quit = True
            return
        if self.help_mode:
            return
        if key in (curses.KEY_F5, ord('e')):
            self.do_scan()
            return
        if key == ord('E'):
            self.do_enable_all()
            return
        if key == ord('D'):
            self.do_disable_all()
            return
        if key in (curses.KEY_DOWN, ord('j')):
            if motors:
                self.cursor = (self.cursor + 1) % len(motors)
                self._select(motors[self.cursor])
            return
        if key in (curses.KEY_UP, ord('k')):
            if motors:
                self.cursor = (self.cursor - 1) % len(motors)
                self._select(motors[self.cursor])
            return
        if ord('1') <= key <= ord('6'):
            slot = key - ord('1')
            for m in motors:
                if m.joint_slot == slot:
                    self._select(m)
                    return
            self.status = f"J{slot+1} 无电机"
            return
        if key in (10, 13):
            if motors:
                self._select(motors[self.cursor])
            return
        if key == ord('a'):
            self.do_arm()
            return
        if key == ord('d'):
            self.do_disarm()
            return
        if key == ord('x'):
            if self.sm.selected_id:
                self.bus.send_payload(self.sm.selected_id,
                                      add_checksum(bytes([F_STOP, 0x98, 0x00])))
                self.status = "已停止选中电机"
            return
        if key in (ord('X'), 27):
            self.do_estop()
            return
        if key == ord('.'):
            self.do_step(1)
            return
        if key == ord(','):
            self.do_step(-1)
            return
        if key == ord('+'):
            self.sm.step_size_deg = min(5.0, self.sm.step_size_deg + 0.5)
            self.status = f"步进幅值 {self.sm.step_size_deg:.1f}°"
            return
        if key == ord('-'):
            self.sm.step_size_deg = max(0.1, self.sm.step_size_deg - 0.5)
            self.status = f"步进幅值 {self.sm.step_size_deg:.1f}°"
            return
        if key == ord('s'):
            self.do_set_speed()
            return
        if key == ord('t'):
            if self.sm.selected_id:
                self._telemetry_once(self.sm.selected_id)
            return
        if key == ord('T'):
            self.continuous_telemetry = not self.continuous_telemetry
            self.status = ("连续遥测开" if self.continuous_telemetry else "连续遥测关")
            return
        if key == ord('p'):
            self.do_params_read()
            return
        if key == ord('m'):
            self.do_params_write()
            return
        if key == ord('i'):
            self.do_id_change()
            return
        if key == ord('r'):
            self.do_re_arm()
            return

    def _quit_confirm(self, s) -> None:
        if s == "QUIT":
            self.quit = True
        else:
            self.status = "取消退出"


HELP_TEXT = """\
键位:
  e/F5 枚举   ↑↓/jk 选择   1-6 跳关节   Enter 选中
  a 臂置(重力关节按 y 确认)    d 失臂置    E 全使能    D 全禁用(输 CONFIRM)
  x 停选中    X/Esc 广播急停(任意时刻, 含输入中)    r 复位急停(按 y)
  . , 微步进 ± (非阻塞, 运动时可随时 X)    + - 调步进幅值    s 限速
  t 单次遥测   T 连续遥测    p 读参数    m 改参数    i 改ID(输 CONFIRM)
  ? 本帮助    q 退出(有使能电机输 QUIT)
安全: 电机默认保持力矩; e_stop 只停不解除力矩; 表自动刷新(约1Hz)"""


def dump_mode(bus: ZdtBus, sm: SafetyMachine, args) -> None:
    """一次性文本枚举."""
    if args.probe is not None:
        from arm_robot.driver.scan import probe_id
        cid = args.probe
        m = probe_id(bus, cid)
        if m is None:
            print(f"0x{cid:02X} 无响应 (可能: 未校准 Not Cal / Response=None / 波特率 / 接线 / 重复ID)")
        else:
            fw = f"{m.fw_ver[0]:#x}/{m.fw_ver[1]:#x}" if m.fw_ver else "-"
            print(f"0x{cid:02X} 在线 FW/HW={fw} flag=0x{m.flags:02X} {m.note}")
        return

    print("== ZDT CAN 枚举 ==")
    result = scan_bus(bus, id_range=(1, args.scan_max),
                      forced_scheme=None if args.addr_scheme == "auto"
                      else args.addr_scheme)
    sm.set_scan(result.found)
    print(f"scheme: {result.scheme or '?'}   在线电机: {len(result.found)}   扫描范围 1..{args.scan_max}")
    # 只读遥测 (Phase 2 全表)
    from arm_robot.driver.scan import read_telemetry
    for m in result.found.values():
        read_telemetry(bus, m)
    print("J#  ID    在线  位置°   速度RPM 电流mA 温度°C  使能 到位 堵转 编码器[就绪/校准]")
    for cid in sorted(result.found):
        m = result.found[cid]
        jm = JOINTS[m.joint_slot] if m.joint_slot is not None else None
        slot = f"J{m.joint_slot+1}" if jm else "?"
        pos = f"{m.pos_deg:8.1f}" if m.pos_deg is not None else "      -"
        vel = f"{m.velocity_rpm:6.1f}" if m.velocity_rpm is not None else "    -"
        cur = f"{m.current_ma:6.0f}" if m.current_ma is not None else "    -"
        tmp = f"{m.temp_c:6.1f}" if m.temp_c is not None else "   -"
        en = "Y" if m.flags & 0x01 else "-"
        arr = "Y" if m.flags & 0x02 else "-"
        stl = "!" if m.flags & 0x04 else "-"
        enc = f"{'R' if m.home_flags & 0x01 else '-'}/{'C' if m.home_flags & 0x02 else '-'}"
        print(f"{slot:<3} 0x{cid:02X}   {pos}  {vel}  {cur}  {tmp}    {en}   {arr}   {stl}    {enc}")
    for w in result.warnings:
        print(f"  ⚠ {w}")
    # 通讯稳定性: 连续 5 轮读全部 flag, 统计掉线/不一致
    print("\n== 通讯稳定性 (连续 5 轮读全部 flag) ==")
    import time as _t
    consistent = True
    for rnd in range(1, 6):
        ok = []
        for cid in sorted(result.found):
            fd = bus.request(cid, add_checksum(bytes([F_READ_FLAG])),
                             F_READ_FLAG, timeout_s=0.1)
            ok.append(fd is not None)
        line = f"  轮 {rnd}: " + " ".join(
            f"0x{cid:02X}:{'✓' if o else '✗'}"
            for cid, o in zip(sorted(result.found), ok))
        print(line)
        if not all(ok):
            consistent = False
        _t.sleep(0.1)
    print("结论:", "通讯稳定 ✓" if consistent else "⚠ 存在掉线/丢帧, 需排查")
    if args.params:
        _dump_params(bus, result)
    if len(result.found) < 6:
        print("  提示: 少于 6 台 → 用 --probe <id> 逐地址排查; 检查缺的电机的 OLED:")
        print("        编码器未校准(Not Cal) / P_Serial=CAN1_MAP / CAN_Baud=500000 /")
        print("        Response≠None / 波特率一致 / CAN 接线与 120Ω 终端 / 是否重复 ID")
    elif not result.found:
        print("  提示: 无响应 → 检查供电/波特率/P_Serial=CAN1_MAP/Response 设置")


def _dump_params(bus, result) -> None:
    """只读读取并打印每个在线电机的 0x42 参数块 (Phase 3 / Gate 3 确认)."""
    from arm_robot.controller.params import (
        CAN_BAUD_LABELS, RESPONSE_LABELS, parse_42_response,
    )
    print("\n== 驱动器参数块 (0x42, 只读) ==")
    for cid in sorted(result.found):
        data = bus.request_multi(cid, add_checksum(bytes([F_READ_PARAMS, 0x6C])),
                                 F_READ_PARAMS, timeout_s=0.5)
        if data is None:
            print(f"  0x{cid:02X}: 参数读取超时")
            continue
        try:
            p = parse_42_response(data)
            print(f"  0x{cid:02X}: MStep={p.mstep}(pulses/rev={p.pulses_per_rev}) "
                  f"CAN_Baud={CAN_BAUD_LABELS.get(p.can_baud, '?')} "
                  f"Checksum={p.checksum} Response={RESPONSE_LABELS.get(p.response, '?')} "
                  f"S_PosTDP={p.pos_tdp} Ma_Limit={p.ma_limit_ma}mA "
                  f"Vm_Limit={p.vm_limit_rpm}RPM Clog_Pro={p.clog_pro}")
            if p.warning:
                print(f"          ⚠ {p.warning}")
        except ValueError as e:
            print(f"  0x{cid:02X}: 解析失败 {e}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="ZDT 六轴 CAN 直连安全调试面板 — 绕过 STM32, 仅限 bring-up")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--addr-scheme", choices=["auto", "firmware", "pc"],
                    default="auto")
    ap.add_argument("--scan-max", type=int, default=8,
                    help="扫描地址上限 (默认 8; 电机 ID 可能 >8 时调大, 如 16)")
    ap.add_argument("--probe", type=lambda x: int(x, 0), default=None,
                    help="dump 模式单地址探测 (如 --probe 0x07), 排查缺失电机")
    ap.add_argument("--watchdog", type=float, default=3.0,
                    help="看门狗秒数 (电机使能时)")
    ap.add_argument("--params", action="store_true",
                    help="dump 模式附带只读读取 0x42 参数块 (Phase 3)")
    ap.add_argument("--dump", action="store_true", help="一次性文本模式")
    args = ap.parse_args()

    print("⚠ 本工具绕过 STM32 直接控制电机 — 无固件限位保护, 请确保手臂安全且有人值守")
    if not sys.stdin.isatty() and not args.dump:
        print("非 TTY 环境, 用 --dump 文本模式")
        return 2

    from arm_robot.driver.can_transport import SocketCanTransport
    latch: dict = {"estop": False}
    bus = ZdtBus(SocketCanTransport(args.iface), watchdog_s=args.watchdog,
                 on_watchdog=lambda: latch.__setitem__("estop", True))
    sm = SafetyMachine()
    try:
        bus.open()
        if args.dump:
            dump_mode(bus, sm, args)
            return 0
        curses.wrapper(lambda stdscr: PanelApp(stdscr, bus, sm, args, latch).loop())
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
