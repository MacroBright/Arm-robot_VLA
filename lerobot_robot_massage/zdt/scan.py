"""ZDT 总线枚举 — 探测在线的驱动器并裁决寻址 scheme.

驱动器**没有"读 ID"命令**, 只能逐个候选地址探测. 固件与 PC 配置存在寻址冲突
(固件 J1=0x01..J6=0x06, PC 配置 J1=0x02..J6=0x07), 枚举结果用于裁决.

探测策略: 每个候选 ID 用轻量读 (0x1F 版本) 作 gate, 超时即离线; 不探测
地址 0 (广播地址, 只有 ID1 会回复, 会产生假阳性). 全部离线时提示可能是
Response=None 配置 / 波特率不符 / 未供电.
"""
from dataclasses import dataclass, field
from typing import Optional

from .config import FIRMWARE_JOINT_ADDRS, JOINT_ADDRS
from .frames import add_checksum
from .safety import JOINTS, MotorState
from .zdt_bus import ZdtBus

DEFAULT_SCAN_RANGE: tuple[int, int] = (1, 8)   # 覆盖 1..6 与 2..7 之争 + 余量
PROBE_TIMEOUT_S: float = 0.06

F_READ_VER: int = 0x1F
F_READ_FLAG: int = 0x3A
F_READ_POS: int = 0x36
F_READ_VEL: int = 0x35
F_READ_CUR: int = 0x27
F_READ_TEMP: int = 0x39
F_READ_HOME: int = 0x3B


def _sign(data: bytes) -> int:
    """响应符号字节: 00=正, 01=负 (手册 0x36/0x35/0x39 均如此)."""
    return -1 if len(data) > 1 and data[1] == 0x01 else 1


def read_telemetry(bus: ZdtBus, motor: MotorState,
                   timeout_s: float = 0.1) -> MotorState:
    """只读填充 Phase 2 遥测字段: 位置/速度/电流/温度/编码器状态 (0x36/35/27/39/3B)."""
    for func, attr, parse in (
        (F_READ_POS, "pos_deg", lambda d: _sign(d) * ((d[2] << 16 | d[3] << 8 | d[4]) / 10.0)),
        (F_READ_VEL, "velocity_rpm", lambda d: _sign(d) * ((d[2] << 8 | d[3]) / 10.0)),
        (F_READ_CUR, "current_ma", lambda d: float(d[1] << 8 | d[2])),
        (F_READ_TEMP, "temp_c", lambda d: _sign(d) * float(d[2])),
    ):
        data = bus.request(motor.can_id, add_checksum(bytes([func])), func,
                           timeout_s=timeout_s)
        if data is not None and len(data) > 3:
            setattr(motor, attr, parse(data))
        else:
            setattr(motor, attr, None)
    home = bus.request(motor.can_id, add_checksum(bytes([F_READ_HOME])),
                       F_READ_HOME, timeout_s=timeout_s)
    motor.home_flags = home[1] if home is not None and len(home) > 1 else 0
    return motor


@dataclass
class ScanResult:
    found: dict[int, MotorState] = field(default_factory=dict)
    scheme: Optional[str] = None          # "firmware" | "pc" | None
    warnings: list[str] = field(default_factory=list)


def probe_id(bus: ZdtBus, can_id: int, timeout_s: float = PROBE_TIMEOUT_S,
             retries: int = 0) -> Optional[MotorState]:
    """探测单个 ID: 0x1F 版本为 gate, 再读 0x3A 标志 + 0x36 位置. 离线返回 None."""
    for _ in range(retries + 1):
        data = bus.request(can_id, add_checksum(bytes([F_READ_VER])),
                           F_READ_VER, timeout_s=timeout_s)
        if data is None:
            continue
        fw = data[1] if len(data) > 2 else 0
        hw = data[2] if len(data) > 3 else 0
        flags = 0
        flag_data = bus.request(can_id, add_checksum(bytes([F_READ_FLAG])),
                                F_READ_FLAG, timeout_s=timeout_s)
        if flag_data is not None and len(flag_data) > 1:
            flags = flag_data[1]
        pos_data = bus.request(can_id, add_checksum(bytes([F_READ_POS])),
                               F_READ_POS, timeout_s=timeout_s)
        note = ""
        if pos_data is None:
            note = "0x36 无响应 (Response=None?)"
        return MotorState(can_id=can_id, online=True, fw_ver=(fw, hw),
                          flags=flags, tracked_deg=0.0, note=note)
    return None


def resolve_scheme(found_ids: set[int]) -> tuple[Optional[str], list[str]]:
    """裁决寻址 scheme + 生成缺失/异常告警.

    返回 (scheme, warnings). scheme 为 "firmware"|"pc"|None(歧义).
    """
    ids = set(found_ids)
    warnings: list[str] = []
    fw = set(FIRMWARE_JOINT_ADDRS)
    pc = set(JOINT_ADDRS)

    if len(ids) > 6:
        return None, [f"发现 {len(ids)} 个电机, 超过 6 关节, 需手动映射"]

    if ids == fw:
        scheme = "firmware"
    elif ids == pc:
        scheme = "pc"
    else:
        # 部分集合: 仅当严格属于其中一侧且非另一侧子集时才自动裁决;
        # 两侧都是子集 (如 {2..6}) → 歧义, 手动指定, 避免错标关节槽.
        fw_sub = ids.issubset(fw)
        pc_sub = ids.issubset(pc)
        if fw_sub and not pc_sub:
            scheme = "firmware"
        elif pc_sub and not fw_sub:
            scheme = "pc"
        else:
            scheme = None
            fw_missing = sorted(fw - ids)
            pc_missing = sorted(pc - ids)
            hints = []
            if fw_missing:
                hints.append("firmware侧缺: " + ", ".join(f"J{i+1}(0x{e:02X})"
                              for i, e in enumerate(FIRMWARE_JOINT_ADDRS) if e in fw_missing))
            if pc_missing:
                hints.append("PC侧缺: " + ", ".join(f"J{i+1}(0x{e:02X})"
                              for i, e in enumerate(JOINT_ADDRS) if e in pc_missing))
            warnings.append(
                "地址分配模糊 (缺轴/部分匹配): 固件 1..6 vs PC 2..7 均不完全确定 — "
                "用 --addr-scheme 指定或手动映射; " + " ".join(hints))

    if scheme == "firmware":
        expected = FIRMWARE_JOINT_ADDRS
        slot_of = lambda cid: cid - 1
    elif scheme == "pc":
        expected = JOINT_ADDRS
        slot_of = lambda cid: cid - 2
    else:
        expected = None
        slot_of = None

    if expected is not None:
        for i, eid in enumerate(expected):
            if eid not in ids:
                warnings.append(f"J{i+1} MISSING (期望 0x{eid:02X})")
        for cid in sorted(ids):
            if cid not in expected:
                warnings.append(f"0x{cid:02X} 不在 6 关节映射内 (unexpected)")
    return scheme, warnings


def scan_bus(bus: ZdtBus, id_range: Optional[tuple[int, int]] = None,
             timeout_s: float = PROBE_TIMEOUT_S,
             forced_scheme: Optional[str] = None) -> ScanResult:
    """扫描 id_range (含两端) 所有候选地址, 返回 ScanResult.

    forced_scheme: 手动指定 scheme ("firmware"/"pc"), 忽略自动裁决.
    """
    lo, hi = id_range or DEFAULT_SCAN_RANGE
    result = ScanResult()
    for cid in range(lo, hi + 1):
        if cid == 0:
            continue                      # 广播地址, 不探测
        ms = probe_id(bus, cid, timeout_s=timeout_s)
        if ms is not None:
            result.found[cid] = ms

    if not result.found:
        result.scheme = None
        result.warnings.append(
            "总线无响应: 检查供电 / 波特率 / 驱动板 P_Serial=CAN1_MAP / "
            "Response 非 None; 可用手动添加 ID 做只发操作")
        return result

    scheme, warnings = resolve_scheme(set(result.found))
    if forced_scheme:
        if scheme and scheme != forced_scheme:
            warnings.append(f"自动裁决为 {scheme}, 强制使用 {forced_scheme}")
        scheme = forced_scheme
    result.scheme = scheme
    result.warnings = warnings

    if scheme == "firmware":
        _assign_slots(result.found, slot_of=lambda cid: cid - 1)
    elif scheme == "pc":
        _assign_slots(result.found, slot_of=lambda cid: cid - 2)
    # scheme=None → joint_slot 保持 None, 面板走手动映射
    return result


def _assign_slots(found: dict[int, MotorState], slot_of) -> None:
    """按 scheme 映射关节槽, 并把跟踪角初始化为该关节初始位姿角.

    ⚠ tracked_deg 默认 0.0; 若不在枚举时改为初始角, 首次步进会被软限位 clamp
    放大 (如 J2 下限 90°, +1° 请求 target=max(90, 0+1)=90, actual=90° → 电机
    持续运转). 见 test_safety::test_step_never_amplified_beyond_request.
    """
    for cid, m in found.items():
        slot = slot_of(cid)
        m.joint_slot = slot
        if 0 <= slot < len(JOINTS):
            m.tracked_deg = JOINTS[slot].init_deg
