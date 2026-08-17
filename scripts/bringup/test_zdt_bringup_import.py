"""zdt_bringup CLI 与顶层导出冒烟 (直接运行, 无需硬件)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from lerobot_robot_massage import ZdtConfig, ZdtController  # noqa: E402


def test_top_level_export():
    assert ZdtController is not None and ZdtConfig is not None


def test_cli_help_exits_zero():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "bringup" / "zdt_bringup.py"),
                        "--help"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_cli_status_parses_args():
    # 无 CAN 硬件: connect 会抛错, 但 argparse 应正确解析 (stderr 不应含 "usage" 用法错误)
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "bringup" / "zdt_bringup.py"),
                        "status", "--n", "1"], capture_output=True, text=True, timeout=15)
    assert "usage" not in r.stderr, r.stderr


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
