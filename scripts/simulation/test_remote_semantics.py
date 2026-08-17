"""remote_semantics 纯函数单测（R_A 无 pytest，直接运行）。

用法: conda activate smolvla && python scripts/test_remote_semantics.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from remote_semantics import parse_remote_event


def test_vx_negated():
    v_lin, j4, j5, j6 = parse_remote_event([1, 0, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(v_lin, [-1, 0, 0])
    assert j4 == 0.0 and j5 == 0.0 and j6 == 0.0


def test_vy_direct():
    v_lin, _, _, _ = parse_remote_event([0, 0.5, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(v_lin, [0, 0.5, 0])


def test_vz_from_p4p5():
    # vz = (p4 - p5)/2: p4=0.6, p5=0.2 → 0.2
    v_lin, _, _, _ = parse_remote_event([0, 0, 0, 0, 0.6, 0.2, 0])
    np.testing.assert_allclose(v_lin, [0, 0, 0.2])


def test_j6_from_p2():
    v_lin, j4, j5, j6 = parse_remote_event([0, 0, 0.8, 0, 0, 0, 0])
    assert j6 == 0.8 and j5 == 0.0 and j4 == 0.0


def test_j5_negated_from_p3():
    v_lin, j4, j5, j6 = parse_remote_event([0, 0, 0, 0.4, 0, 0, 0])
    assert j5 == -0.4 and j6 == 0.0 and j4 == 0.0


def test_j4_from_p6():
    # p6 → J4 (仿真扩展通道), 且 p6 不影响 v_lin/j5/j6
    v_lin, j4, j5, j6 = parse_remote_event([0, 0, 0, 0, 0, 0, 0.5])
    assert j4 == 0.5 and j5 == 0.0 and j6 == 0.0
    np.testing.assert_allclose(v_lin, [0, 0, 0])
    # p6 与线速度/其他关节正交
    v_lin, j4, j5, j6 = parse_remote_event([1, 1, 1, 1, 1, 1, 1])
    np.testing.assert_allclose(v_lin, [-1, 1, 0])
    assert j4 == 1.0 and j5 == -1.0 and j6 == 1.0


def test_client_roundtrip():
    # 客户端公式 p0=-vx p1=vy p2=j6 p3=-j5 p4=vz p5=-vz p6=j4 → 应还原原值
    vx, vy, vz, j4, j5, j6 = 0.7, -0.3, 0.5, 0.6, 0.4, -0.9
    vals = [-vx, vy, j6, -j5, vz, -vz, j4]
    v_lin, j4_out, j5_out, j6_out = parse_remote_event(vals)
    np.testing.assert_allclose(v_lin, [vx, vy, vz])
    assert j4_out == j4 and j5_out == j5 and j6_out == j6


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
    sys.exit(1 if failed else 0)
