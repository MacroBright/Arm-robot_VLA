"""共享测试运行器 (无 pytest, 对齐 scripts/test_remote_semantics.py 约定)."""
import sys


def run_all(module_globals: dict) -> None:
    """遍历 globals() 中 test_* 函数并运行, 打印 PASS/FAIL, 失败退出 1."""
    failed = 0
    for name, fn in sorted(module_globals.items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {exc}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
    sys.exit(1 if failed else 0)
