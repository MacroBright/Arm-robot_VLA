"""RobotStateMachine 状态机 + 枚举硬不变式测试 (spec §5.1, 修订 #6)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.safety import (  # noqa: E402
    MotorState, RobotPhase, RobotStateMachine, SafetyError, verify_enumeration,
)


def _motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06, 0x07)):
    return {a: MotorState(can_id=a, online=True, joint_slot=i)
            for i, a in enumerate(addrs)}


def _happy_machine():
    sm = RobotStateMachine()
    sm.on_connected()
    sm.on_enumerated(_motors())
    sm.on_safe_idle()
    return sm


def test_happy_path_full_cycle():
    sm = RobotStateMachine()
    sm.on_connected()
    sm.on_enumerated(_motors())
    sm.on_safe_idle()
    sm.arm(gravity_confirmed=True)
    sm.enter_teleop()
    assert sm.phase == RobotPhase.TELEOP
    sm.exit_teleop()
    assert sm.phase == RobotPhase.ARMED
    sm.e_stop()
    assert sm.phase == RobotPhase.STOPPED
    sm.re_arm(confirmed=True)
    assert sm.phase == RobotPhase.ENUMERATED
    sm.on_safe_idle()
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_illegal_transition_rejected():
    sm = RobotStateMachine()
    try:
        sm.on_safe_idle()      # DISCONNECTED 直接跳 SAFE_IDLE
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.DISCONNECTED


def test_arm_requires_gravity_confirmation():
    sm = _happy_machine()
    try:
        sm.arm(gravity_confirmed=False)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_arm_requires_safe_idle():
    sm = RobotStateMachine()
    sm.on_connected()
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass


def test_estop_latches():
    sm = _happy_machine()
    sm.arm(gravity_confirmed=True)
    sm.e_stop()
    assert sm.phase == RobotPhase.STOPPED
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("STOPPED 后应拒绝 arm")
    except SafetyError:
        pass
    try:
        sm.re_arm(confirmed=False)
        raise AssertionError("re_arm 无确认应拒绝")
    except SafetyError:
        pass


def test_fault_latches_and_reason():
    sm = _happy_machine()
    sm.fault("J4 MISSING")
    assert sm.phase == RobotPhase.FAULT
    assert sm.fault_reason == "J4 MISSING"
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("FAULT 后应拒绝 arm")
    except SafetyError:
        pass
    sm.re_arm(confirmed=True)
    assert sm.phase == RobotPhase.ENUMERATED


def test_terminal_latches_idempotent():
    # 闩锁幂等: FAULT 后 e_stop 不覆盖成 STOPPED (保留原因); STOPPED 后 fault 保持 STOPPED
    sm = _happy_machine()
    sm.fault("bus dead")
    sm.e_stop()
    assert sm.phase == RobotPhase.FAULT
    sm2 = _happy_machine()
    sm2.e_stop()
    sm2.fault("later fault")
    assert sm2.phase == RobotPhase.STOPPED


def test_disarm_returns_to_safe_idle():
    sm = _happy_machine()
    sm.arm(gravity_confirmed=True)
    sm.enter_teleop()
    sm.disarm()
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_assert_armed_gates():
    sm = _happy_machine()
    try:
        sm.assert_armed()
        raise AssertionError("SAFE_IDLE 不应通过 assert_armed")
    except SafetyError:
        pass
    sm.arm(gravity_confirmed=True)
    sm.assert_armed()
    sm.enter_teleop()
    sm.assert_armed()      # TELEOP 也算 armed
    sm.exit_teleop()       # 回到 ARMED 才能正确测 "非 TELEOP"
    try:
        sm.assert_teleop()  # 非 TELEOP → 拒绝
        raise AssertionError("ARMED 不应通过 assert_teleop")
    except SafetyError:
        pass
    sm.enter_teleop()
    sm.assert_teleop()     # TELEOP 通过


def test_verify_enumeration_ok():
    assert verify_enumeration(_motors()) == []


def test_verify_enumeration_missing_motor():
    problems = verify_enumeration(_motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06)))
    assert any("J6 MISSING" in p for p in problems)


def test_verify_enumeration_duplicate_slot():
    motors = _motors()
    motors[0x08] = MotorState(can_id=0x08, online=True, joint_slot=0)   # 与 0x02 同槽
    problems = verify_enumeration(motors)
    assert any("重复" in p for p in problems)


def test_verify_enumeration_unmapped():
    motors = _motors()
    motors[0x09] = MotorState(can_id=0x09, online=True, joint_slot=None)
    problems = verify_enumeration(motors)
    assert any("未映射" in p for p in problems)


def test_verify_enumeration_offline():
    motors = _motors()
    motors[0x02].online = False
    problems = verify_enumeration(motors)
    assert any("不在线" in p for p in problems)


def test_verify_enumeration_wrong_count():
    problems = verify_enumeration(_motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)))
    assert any("7 台" in p for p in problems)


def test_on_enumerated_hard_invariant_faults():
    # 硬不变式: 缺失任一电机 → on_enumerated 抛 SafetyError (调用方转 FAULT)
    sm = RobotStateMachine()
    sm.on_connected()
    motors = _motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06))   # 缺 J6
    try:
        sm.on_enumerated(motors)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.CONNECTED   # 状态不前进


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {exc}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
