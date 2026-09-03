"""SafetyMachine 安全状态机 + 换算单测."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.safety import (
    JOINTS, MotorState, Phase, SafetyError, SafetyMachine,
)
from lerobot_robot_massage.zdt.testutil import run_all


def _mk_motor(can_id=0x02, slot=0):
    m = MotorState(can_id=can_id, joint_slot=slot, online=True)
    return m


def _mk_machine():
    sm = SafetyMachine()
    sm.set_scan({0x02: _mk_motor(0x02, 0)})   # J1
    return sm


def test_state_machine_walk():
    sm = _mk_machine()
    assert sm.phase == Phase.ENUMERATED
    sm.select(0x02)
    assert sm.phase == Phase.MOTOR_SELECTED
    sm.arm()
    assert sm.phase == Phase.ARMED
    plan = sm.request_step(1.0)
    assert sm.phase == Phase.STEPPING
    assert plan.delta_deg == 1.0 and plan.target_deg == 1.0
    sm.step_complete()
    assert sm.phase == Phase.ARMED


def test_cannot_move_unless_armed():
    sm = _mk_machine()
    try:
        sm.request_step(1.0)
        raise AssertionError("IDLE/ENUMERATED 应拒绝运动")
    except SafetyError:
        pass
    sm.select(0x02)
    try:
        sm.request_step(1.0)      # MOTOR_SELECTED 未臂置
        raise AssertionError("未臂置应拒绝运动")
    except SafetyError:
        pass


def test_estop_latches_from_any_phase():
    sm = _mk_machine()
    sm.select(0x02)
    sm.arm()
    sm.e_stop()
    assert sm.phase == Phase.STOPPED
    assert not sm.motors[0x02].armed
    # 闩锁: 运动拒绝
    try:
        sm.request_step(1.0)
        raise AssertionError("STOPPED 应拒绝运动")
    except SafetyError:
        pass
    # re_arm 需确认
    try:
        sm.re_arm(confirmed=False)
        raise AssertionError("re_arm 未确认应拒绝")
    except SafetyError:
        pass
    sm.re_arm(confirmed=True)
    assert sm.phase == Phase.ENUMERATED


def test_estop_works_even_before_enumeration():
    sm = SafetyMachine()          # IDLE
    sm.e_stop()
    assert sm.phase == Phase.STOPPED


def test_gravity_arm_requires_confirm():
    # J2 (slot 1) 是重力关节
    sm = SafetyMachine()
    sm.set_scan({0x03: _mk_motor(0x03, 1)})
    sm.select(0x03)
    try:
        sm.arm()
        raise AssertionError("重力关节未确认应拒绝臂置")
    except SafetyError:
        pass
    sm.arm(gravity_confirmed=True)     # 显式确认后放行
    assert sm.phase == Phase.ARMED


def test_non_gravity_arm_no_confirm_needed():
    sm = _mk_machine()            # J1 非重力
    sm.select(0x02)
    sm.arm()
    assert sm.phase == Phase.ARMED


def test_clamp_delta_refuses_at_limit():
    jm = JOINTS[0]                # J1 [-1,360]
    # 抵上限: tracked 359.9, +1 → clamp 到 360, 实际 0.1
    ok, actual, target = SafetyMachine.clamp_delta(jm, 359.9, 1.0)
    assert ok and abs(actual - 0.1) < 1e-6 and target == 360.0
    # 已在 360, +1 → 实际 0 → 拒绝
    ok, actual, _ = SafetyMachine.clamp_delta(jm, 360.0, 1.0)
    assert not ok
    # 已在开机余量下界 -1, -1 → 拒绝
    ok, actual, _ = SafetyMachine.clamp_delta(jm, -1.0, -1.0)
    assert not ok


def test_dir_byte_matches_table_all_joints():
    sm = SafetyMachine()
    for jm in JOINTS:
        assert sm.dir_byte_for(jm, 5.0) == jm.dir_pos_byte     # 正方向
        assert sm.dir_byte_for(jm, -5.0) == 1 - jm.dir_pos_byte  # 负方向取反


def test_speed_cap_and_step_cap_enforced():
    sm = _mk_machine()
    sm.select(0x02)
    sm.arm()
    # 步进幅值上限: step_size=1.0, 请求 2.0 → 拒绝
    try:
        sm.request_step(2.0)
        raise AssertionError("步进超幅值应拒绝")
    except SafetyError:
        pass
    # 速度上限固定为 HARD_SPEED_CAP_RPM
    plan = sm.request_step(1.0)
    assert plan.speed_rpm <= 30.0


def test_pulses_match_firmware_formula():
    sm = SafetyMachine()
    jm = JOINTS[0]                # J1 ratio 50
    assert sm.pulses_for(jm, 1.0) == int(round(1.0 * 50 * 3200 / 360))
    jm4 = JOINTS[3]               # J4 ratio 51
    assert sm.pulses_for(jm4, 0.5) == int(round(0.5 * 51 * 3200 / 360))


def test_select_updates_flag_and_rejects_unknown():
    sm = _mk_machine()
    m = sm.select(0x02)
    assert m.selected and sm.selected_id == 0x02
    try:
        sm.select(0x99)
        raise AssertionError("未知 ID 应拒绝")
    except SafetyError:
        pass


def test_step_never_amplified_beyond_request():
    """回归: tracked_deg 越出限位(未初始化/外力搬动)时, clamp 不得把步进放大.

    J2 软限位 [-1,150], tracked_deg=160 (越出上限) → +1° 请求 target=clamp(161)=150,
    actual=150-160=-10 (反向放大). 应拒绝并提示跟踪角越界.
    """
    sm = SafetyMachine()
    m = _mk_motor(0x03, 1)                  # J2 重力关节
    m.tracked_deg = 160.0                    # 越出 J2 上限 150
    sm.set_scan({0x03: m})
    sm.select(0x03)
    sm.arm(gravity_confirmed=True)
    try:
        sm.request_step(1.0)
        raise AssertionError("tracked 越出限位时步进被 clamp 放大, 应拒绝")
    except SafetyError as e:
        assert "越出限位" in str(e)


def test_motor_without_joint_slot_cannot_move():
    sm = SafetyMachine()
    sm.set_scan({0x05: _mk_motor(0x05, None)})   # 未映射关节槽
    sm.select(0x05)
    sm.arm()
    try:
        sm.request_step(1.0)
        raise AssertionError("未映射关节槽应拒绝")
    except SafetyError:
        pass


# ── B 任务: clamp_delta_real / drift_check ───────────────────────────────

def test_clamp_delta_real_same_semantics_as_clamp_delta():
    """clamp_delta_real 与 clamp_delta 同形, 限位 clamp 逻辑一致."""
    jm = JOINTS[0]   # J1 limits (0, 360)
    ok, actual, target = SafetyMachine.clamp_delta_real(jm, 50.0, 10.0)
    assert ok and abs(actual - 10.0) < 1e-9 and abs(target - 60.0) < 1e-9


def test_clamp_delta_real_clamps_to_upper_limit():
    """real=350, delta=20 → target=370 clamp 到 360, actual=10."""
    jm = JOINTS[0]   # (0, 360)
    ok, actual, target = SafetyMachine.clamp_delta_real(jm, 350.0, 20.0)
    assert ok and abs(actual - 10.0) < 1e-9 and abs(target - 360.0) < 1e-9


def test_clamp_delta_real_clamps_to_lower_limit():
    """real=5, delta=-10 → target=-5 clamp 到 -1 (开机余量下界), actual=-6."""
    jm = JOINTS[0]   # J1 (-1, 360)
    ok, actual, target = SafetyMachine.clamp_delta_real(jm, 5.0, -10.0)
    assert ok and abs(actual - (-6.0)) < 1e-9 and abs(target - (-1.0)) < 1e-9


def test_clamp_delta_real_at_boundary_no_move():
    """real=360 (上限), delta=10 → target=360, actual=0 → ok=False."""
    jm = JOINTS[0]   # (0, 360)
    ok, actual, target = SafetyMachine.clamp_delta_real(jm, 360.0, 10.0)
    assert not ok and abs(actual) < 1e-9


def test_clamp_delta_real_uses_real_not_tracked():
    """关键: real=10 (外力搬动), delta=80 → target=90 (基于 real, 不是 tracked=0)."""
    jm = JOINTS[0]
    ok, actual, target = SafetyMachine.clamp_delta_real(jm, 10.0, 80.0)
    assert ok and abs(target - 90.0) < 1e-9 and abs(actual - 80.0) < 1e-9
    # 对比: clamp_delta 用 tracked=0 会得到相同 target, 但语义不同
    # (clamp_delta_real 的 10 是真实位置, clamp_delta 的 10 是命令积分)


def test_drift_check_no_drift():
    """tracked == real → ok=True, drift=0."""
    jm = JOINTS[0]
    ok, drift = SafetyMachine.drift_check(jm, 50.0, 50.0)
    assert ok and abs(drift) < 1e-9


def test_drift_check_detects_drift():
    """tracked=50, real=55 → drift=5 > 2° → ok=False."""
    jm = JOINTS[0]
    ok, drift = SafetyMachine.drift_check(jm, 50.0, 55.0, threshold_deg=2.0)
    assert not ok and abs(drift - 5.0) < 1e-9


def test_drift_check_custom_threshold():
    """drift=1.5, threshold=1 → 超限; threshold=2 → 未超限."""
    jm = JOINTS[0]
    ok_strict, _ = SafetyMachine.drift_check(jm, 50.0, 51.5, threshold_deg=1.0)
    ok_loose, _ = SafetyMachine.drift_check(jm, 50.0, 51.5, threshold_deg=2.0)
    assert not ok_strict and ok_loose


def test_drift_check_negative_drift():
    """tracked=55, real=50 → drift=5 (绝对值, 不带符号)."""
    jm = JOINTS[0]
    ok, drift = SafetyMachine.drift_check(jm, 55.0, 50.0)
    assert not ok and abs(drift - 5.0) < 1e-9


if __name__ == "__main__":
    run_all(globals())
