# ZDT CAN 驱动 + 控制器 + Bring-up 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 6×ZDT 闭环步进驱动通过 PC 直连 CAN（SocketCAN can0, 500k）受控：单轴步进、状态读取、急停、软复位，并接入 LeRobot `MassageRobot` 数据采集。

**Architecture:** 四层——`config.py`（常量/配置）→ `frames.py`（纯函数帧编解码）→ `can_transport.py`（SocketCAN 抽象，惰性导入 python-can）→ `zdt_driver.py`（命令语义+超时重试）→ `controller.py`（`ZdtController`，接口与 `SerialProtocol` 对齐，含安全层）。全部纯 Python + 无 x86 专属依赖，测试遵循仓库约定（无 pytest，直接运行断言脚本）。

**Tech Stack:** Python ≥3.10 · numpy · python-can[socketcan]（仅 SocketCanTransport 惰性导入）· Linux SocketCAN（gs_usb CANable 类适配器）

**依据 spec:** `docs/superpowers/specs/2026-08-15-pc-can-direct-control-design.md` §1-§3、§4.1、§7（本计划只覆盖"让机械臂动起来 + LeRobot 采集"，即 spec 的关节级部分；笛卡尔遥操 IK/edge 部署为后续计划）。

## Global Constraints

- **Python ≥ 3.10**；`requires-python = ">=3.10"`（对齐 `lerobot_robot_massage/pyproject.toml`）
- **测试约定**：无 pytest。每个 `test_*.py` 直接运行（`python <file>`），`__main__` 遍历 `globals()` 的 `test_*` 函数、打印 `PASS`/`FAIL`、失败退出码 1 —— 完全对齐 `scripts/test_remote_semantics.py` 的尾部运行器
- **python-can 惰性导入**：`can_transport.py` 模块顶层**不** `import can`；只在 `SocketCanTransport` 方法内部导入，保证无 python-can 环境也能导入包内其他模块
- **可移植性**：禁止 x86 专属依赖；所有设备路径/接口名走 `ZdtConfig` 配置（未来香橙派 aarch64 零重写）
- **代码风格**：PEP 8 + 类型注解 + Google style docstring（中文注释允许，对齐项目 CLAUDE.md）
- **提交**：Conventional Commits（`feat:` / `fix:` / `test:` / `docs:`）
- **帧约定**（spec §2.1 已核实，写死）：扩展帧 ID = `(addr<<8)|seq`；数据段 = `[功能码, 参数..., 0x6B]`；参数 >7 字节拆多帧（每帧重复功能码）
- **关节→地址**：J1→0x02 … J6→0x07（`config.JOINT_ADDRS`）

---

### Task 1: `config.py` — 常量与配置

**Files:**
- Create: `lerobot_robot_massage/zdt/config.py`
- Create: `lerobot_robot_massage/zdt/__init__.py`（先建空包标记，Task 6 补内容）
- Test: `lerobot_robot_massage/zdt/test_config.py`

**Interfaces:**
- Consumes: 无（本任务建立包骨架）
- Produces:
  - `JOINT_ADDRS: list[int]` — `[0x02, 0x03, 0x04, 0x05, 0x06, 0x07]`
  - `CHECKSUM: int` — `0x6B`
  - 功能码常量 `F_ENABLE=0xF3, F_STOP=0xFE, F_POS=0xFB, F_VEL=0xF6, F_READ_POS=0x36, F_READ_CUR=0x27, F_ARRIVED=0xFD`
  - `POS_SCALE=10.0, VEL_SCALE=10.0`
  - `DEFAULT_LIMITS: list[tuple[float,float]]` — 6 项限位表
  - `INIT_POSE_DEG: list[float]` — `[90.0, 45.0, 90.0, 90.0, 0.0, 0.0]`
  - `ZdtConfig` dataclass — `channel="can0"`, `bitrate=500_000`, `timeout_s=0.1`, `retries=3`, `speed_rpm=60.0`, `watchdog_s=0.5`, `joint_addrs`, `limits`

- [ ] **Step 1: 创建包骨架（空 `__init__.py`）**

```bash
mkdir -p lerobot_robot_massage/zdt
touch lerobot_robot_massage/zdt/__init__.py
```

- [ ] **Step 2: 写失败测试** `lerobot_robot_massage/zdt/test_config.py`

```python
"""config 常量与配置单测 (直接运行: python lerobot_robot_massage/zdt/test_config.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lerobot_robot_massage.zdt.config import (
    CHECKSUM, DEFAULT_LIMITS, INIT_POSE_DEG, JOINT_ADDRS, POS_SCALE,
    VEL_SCALE, ZdtConfig, F_ENABLE, F_POS, F_READ_POS, F_READ_CUR,
    F_STOP, F_VEL, F_ARRIVED,
)


def test_joint_addrs_mapping():
    assert JOINT_ADDRS == [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]


def test_checksum_and_funcs():
    assert CHECKSUM == 0x6B
    assert (F_ENABLE, F_STOP, F_POS, F_VEL, F_READ_POS, F_READ_CUR, F_ARRIVED) == (
        0xF3, 0xFE, 0xFB, 0xF6, 0x36, 0x27, 0xFD)


def test_scales():
    assert POS_SCALE == 10.0 and VEL_SCALE == 10.0


def test_limits_len_and_order():
    assert len(DEFAULT_LIMITS) == 6
    assert DEFAULT_LIMITS[0] == (0.0, 360.0)   # J1 shoulder_pan
    assert DEFAULT_LIMITS[2] == (-90.0, 90.0)  # J3 elbow_flex


def test_init_pose():
    assert INIT_POSE_DEG == [90.0, 45.0, 90.0, 90.0, 0.0, 0.0]


def test_zdtconfig_defaults():
    c = ZdtConfig()
    assert c.channel == "can0"
    assert c.bitrate == 500_000
    assert c.joint_addrs == JOINT_ADDRS
    assert c.limits == DEFAULT_LIMITS
    assert c.speed_rpm == 60.0 and c.watchdog_s == 0.5


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
```

- [ ] **Step 3: 运行确认失败**

Run: `python lerobot_robot_massage/zdt/test_config.py`
Expected: `FAIL` (ImportError) — 因为 config 不存在。

- [ ] **Step 4: 写实现** `lerobot_robot_massage/zdt/config.py`

```python
"""ZDT 驱动常量与配置.

组帧约定 (参考固件 can.c can_SendCmd, spec §2.1):
  扩展帧 ID = (地址<<8) | 包序号
  数据段    = [功能码, 参数..., 0x6B]
  参数 >7 字节 → 拆多帧, 每帧重复功能码
"""
from dataclasses import dataclass, field

# 关节→CAN 地址 (J1→02 ... J6→07), 帧 ID 高字节
JOINT_ADDRS: list[int] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]

# 帧校验字节 (数据段末字节固定)
CHECKSUM: int = 0x6B

# ZDT 功能码
F_ENABLE: int = 0xF3      # 使能/失能
F_STOP: int = 0xFE        # 立即停止
F_POS: int = 0xFB         # 直通限速位置
F_VEL: int = 0xF6         # 速度模式
F_READ_POS: int = 0x36    # 读实时位置
F_READ_CUR: int = 0x27    # 读相电流
F_ARRIVED: int = 0xFD     # 到位帧 (数据段[0])

# 换算倍率
POS_SCALE: float = 10.0   # 位置 ×10
VEL_SCALE: float = 10.0   # 速度 RPM×10

# 关节限位表 (度) — 真机扫掠后修正 (spec §2.2)
DEFAULT_LIMITS: list[tuple[float, float]] = [
    (0.0, 360.0),    # J1 shoulder_pan
    (90.0, 180.0),   # J2 shoulder_lift
    (-90.0, 90.0),   # J3 elbow_flex
    (-90.0, 90.0),   # J4 wrist_flex
    (0.0, 90.0),     # J5 wrist_roll
    (0.0, 360.0),    # J6 gripper
]

# 软复位初始位 (与固件 soft_reset 一致)
INIT_POSE_DEG: list[float] = [90.0, 45.0, 90.0, 90.0, 0.0, 0.0]


@dataclass
class ZdtConfig:
    channel: str = "can0"
    bitrate: int = 500_000
    timeout_s: float = 0.1
    retries: int = 3
    speed_rpm: float = 60.0
    watchdog_s: float = 0.5
    joint_addrs: list[int] = field(default_factory=lambda: list(JOINT_ADDRS))
    limits: list[tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_LIMITS))
```

- [ ] **Step 5: 运行确认通过**

Run: `python lerobot_robot_massage/zdt/test_config.py`
Expected: `ALL PASS`

- [ ] **Step 6: 提交**

```bash
git add lerobot_robot_massage/zdt/__init__.py lerobot_robot_massage/zdt/config.py lerobot_robot_massage/zdt/test_config.py
git commit -m "feat(zdt): ZDT 常量与配置 (地址映射/功能码/限位表)"
```

---

### Task 2: `frames.py` — 纯函数帧编解码

**Files:**
- Create: `lerobot_robot_massage/zdt/frames.py`
- Test: `lerobot_robot_massage/zdt/test_frames.py`

**Interfaces:**
- Consumes: `config.CHECKSUM`, `config.POS_SCALE`, `config.VEL_SCALE`
- Produces:
  - `@dataclass CanFrame` — `arbitration_id:int, data:bytes, is_extended_id:bool=True`
  - `add_checksum(body: bytes) -> bytes` — 末字节非 0x6B 时附加
  - `verify_checksum(data: bytes) -> bool`
  - `payload_chunks(payload: bytes) -> list[bytes]` — 拆包（功能码重复）
  - `encode_frame(addr: int, payload: bytes) -> list[CanFrame]` — ID=(addr<<8)|seq
  - `parse_frame(frame: CanFrame) -> tuple[int,int,bytes]` — (addr, seq, data)
  - `encode_pos3(pos_deg: float) -> bytes` / `decode_pos3(data3: bytes, sign: int=1) -> float`
  - `encode_vel2(rpm: float) -> bytes` / `decode_vel2(data2: bytes) -> float`

- [ ] **Step 1: 写失败测试** `lerobot_robot_massage/zdt/test_frames.py`

```python
"""frames 纯函数单测 (直接运行)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lerobot_robot_massage.zdt.frames import (
    CanFrame, add_checksum, decode_pos3, decode_vel2, encode_frame,
    encode_pos3, encode_vel2, parse_frame, payload_chunks, verify_checksum,
)


def test_add_checksum():
    assert add_checksum(b"\x36") == b"\x36\x6b"
    # 已以 0x6B 结尾不重复
    assert add_checksum(b"\x36\x6b") == b"\x36\x6b"


def test_verify_checksum():
    assert verify_checksum(b"\x36\x6b") is True
    assert verify_checksum(b"\x36\x00") is False
    assert verify_checksum(b"") is False


def test_payload_chunks_single():
    # 参数 ≤7 → 1 帧
    chunks = payload_chunks(b"\x36\x6b")
    assert chunks == [b"\x36\x6b"]


def test_payload_chunks_multi():
    # 0xFB 命令体: FB + 9 参数 → 2 帧, 功能码 FB 重复
    body = bytes([0xFB, 0x01]) + encode_vel2(60.0) + encode_pos3(90.0) + b"\x0a\x00\x6b"
    chunks = payload_chunks(body)
    assert len(chunks) == 2
    assert chunks[0][0] == 0xFB and len(chunks[0]) == 8
    assert chunks[1][0] == 0xFB and len(chunks[1]) == 3


def test_encode_frame_ids():
    frames = encode_frame(0x05, b"\x36\x6b")
    assert len(frames) == 1
    assert frames[0].arbitration_id == (0x05 << 8)
    assert frames[0].is_extended_id is True

    # 多包: 包序号递增
    body = bytes([0xFB, 0x01]) + encode_vel2(60.0) + encode_pos3(90.0) + b"\x0a\x00\x6b"
    frames = encode_frame(0x05, body)
    assert [f.arbitration_id for f in frames] == [0x0500, 0x0501]


def test_parse_frame_roundtrip():
    f = CanFrame(arbitration_id=(0x03 << 8) | 1, data=b"\xfb\x00")
    assert parse_frame(f) == (0x03, 1, b"\xfb\x00")


def test_pos_roundtrip():
    for deg in (0.0, 90.0, 360.0, 7.5):
        assert abs(decode_pos3(encode_pos3(deg)) - deg) < 0.001, f"deg={deg}"


def test_pos_negative():
    # 负角度: 幅值编码 + decode 用 sign=-1
    assert encode_pos3(-45.0)[0] & 0x80            # 符号位在最高位
    assert abs(decode_pos3(encode_pos3(-45.0)[1:], -1) - 45.0) < 0.001


def test_vel_roundtrip():
    assert abs(decode_vel2(encode_vel2(600.0)) - 600.0) < 0.001


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
```

> 注：`encode_pos3` 把符号编码进 3 字节最高位（符号-幅值约定，bring-up candump 核实）。`decode_pos3(data3, sign)` 的 `sign` 参数对应 ZDT 回帧里**独立的符号字节**（spec §2 风险表）。若 bring-up 发现符号在别处，只改这两处，单测隔离。

- [ ] **Step 2: 运行确认失败**

Run: `python lerobot_robot_massage/zdt/test_frames.py`
Expected: `FAIL` (ImportError — frames 不存在)

- [ ] **Step 3: 写实现** `lerobot_robot_massage/zdt/frames.py`

```python
"""ZDT 帧编解码 — 纯函数, 无硬件/无 python-can 依赖 (spec §2.1)."""
from dataclasses import dataclass

from .config import CHECKSUM, POS_SCALE, VEL_SCALE


@dataclass
class CanFrame:
    """CAN 帧的轻量表示 (与 python-can Message 解耦)."""
    arbitration_id: int
    data: bytes
    is_extended_id: bool = True


def add_checksum(body: bytes) -> bytes:
    """附加 0x6B 校验字节; 若末字节已是 0x6B 则不重复."""
    if body and body[-1] == CHECKSUM:
        return bytes(body)
    return bytes(body) + bytes([CHECKSUM])


def verify_checksum(data: bytes) -> bool:
    """末字节 == 0x6B?"""
    return len(data) > 0 and data[-1] == CHECKSUM


def payload_chunks(payload: bytes) -> list[bytes]:
    """把 [功能码, 参数..., 0x6B] 拆成多帧数据段.

    每帧 = [功能码 + ≤7 参数], DLC≤8; 参数 >7 字节拆多帧 (功能码重复,
    包序号由 encode_frame 的 ID 低字节编码).
    """
    if not payload:
        return []
    func, rest = payload[0], payload[1:]
    return [bytes([func]) + rest[i:i + 7] for i in range(0, len(rest), 7)]


def encode_frame(addr: int, payload: bytes) -> list[CanFrame]:
    """ZDT 命令 → CAN 帧列表. 扩展帧 ID = (addr<<8)|seq."""
    return [
        CanFrame(arbitration_id=(addr << 8) | seq, data=chunk)
        for seq, chunk in enumerate(payload_chunks(payload))
    ]


def parse_frame(frame: CanFrame) -> tuple[int, int, bytes]:
    """回帧 → (addr, seq, data)."""
    return frame.arbitration_id >> 8, frame.arbitration_id & 0xFF, frame.data


# ── 参数编解码 ──────────────────────────────────────────────

def encode_pos3(pos_deg: float) -> bytes:
    """位置(°)×10 → 3 字节大端, 符号-幅值 (最高位=符号).

    约定需 bring-up candump 核实 (ZDT 文档未明示命令侧符号位, spec §9 风险表).
    """
    q = int(round(pos_deg * POS_SCALE))
    sign = 0x80 if q < 0 else 0x00
    mag = abs(q) & 0x7FFFFF
    return bytes([sign | (mag >> 16) & 0xFF, (mag >> 8) & 0xFF, mag & 0xFF])


def decode_pos3(data3: bytes, sign: int = 1) -> float:
    """3 字节位置(×10) + 符号字节 → 度."""
    v = ((data3[0] & 0x7F) << 16) | (data3[1] << 8) | data3[2]
    return v / POS_SCALE * sign


def encode_vel2(rpm: float) -> bytes:
    """转速(RPM)×10 → 2 字节大端."""
    q = int(round(rpm * VEL_SCALE)) & 0xFFFF
    return bytes([q >> 8, q & 0xFF])


def decode_vel2(data2: bytes) -> float:
    """2 字节速度(×10) → RPM."""
    return ((data2[0] << 8) | data2[1]) / VEL_SCALE
```

- [ ] **Step 4: 运行确认通过**

Run: `python lerobot_robot_massage/zdt/test_frames.py`
Expected: `ALL PASS`

- [ ] **Step 5: 提交**

```bash
git add lerobot_robot_massage/zdt/frames.py lerobot_robot_massage/zdt/test_frames.py
git commit -m "feat(zdt): 纯函数帧编解码 (校验/拆包/ID/位置速度换算)"
```

---

### Task 3: `can_transport.py` + 测试辅助

**Files:**
- Create: `lerobot_robot_massage/zdt/can_transport.py`
- Create: `lerobot_robot_massage/zdt/testutil.py`（共享运行器）
- Create: `lerobot_robot_massage/zdt/fakes.py`（`FakeTransport`）
- Test: `lerobot_robot_massage/zdt/test_can_transport.py`

**Interfaces:**
- Consumes: `frames.CanFrame`
- Produces:
  - `class CanTransport(ABC)` — `open()`, `close()`, `send(frame)`, `recv(timeout_s)->Optional[CanFrame]`
  - `class SocketCanTransport(CanTransport)` — 惰性导入 python-can
  - `testutil.run_all(globals()) -> None` — 共享测试运行器
  - `class FakeTransport(CanTransport)` — `sent:list[CanFrame]`, `sent_ids:list[int]`, `inject(addr, func, body)`

- [ ] **Step 1: 写共享运行器** `lerobot_robot_massage/zdt/testutil.py`

```python
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
```

（后续 test_*.py 尾部统一为 `if __name__ == "__main__": run_all(globals())`。）

- [ ] **Step 2: 写失败测试** `lerobot_robot_massage/zdt/test_can_transport.py`

```python
"""CanTransport 抽象契约测试 + SocketCanTransport 惰性导入验证."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lerobot_robot_massage.zdt.can_transport import CanTransport, SocketCanTransport
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.frames import CanFrame
from lerobot_robot_massage.zdt.testutil import run_all


def test_fake_transport_open_close():
    t = FakeTransport()
    t.open()
    assert t.opened
    t.close()
    assert t.closed


def test_fake_transport_send_recv_roundtrip():
    t = FakeTransport()
    t.inject(0x03, 0x36, b"\x01\x2c\x01\x6b")
    frame = t.recv(0.01)
    assert frame is not None
    assert frame.arbitration_id == 0x0300
    assert frame.data == bytes([0x36, 0x01, 0x2c, 0x01, 0x6b])


def test_fake_transport_recv_empty_returns_none():
    t = FakeTransport()
    assert t.recv(0.001) is None


def test_socketcan_transport_constructs_without_python_can():
    # 构造不抛错; python-can 缺失时应由 open() 抛 ImportError
    t = SocketCanTransport(channel="can0", bitrate=500_000)
    assert t.channel == "can0"
    assert isinstance(t, CanTransport)


def test_can_transport_is_abstract():
    try:
        CanTransport()
        raise AssertionError("CanTransport 应不可实例化")
    except TypeError:
        pass


if __name__ == "__main__":
    run_all(globals())
```

- [ ] **Step 3: 运行确认失败**

Run: `python lerobot_robot_massage/zdt/test_can_transport.py`
Expected: `FAIL` (ImportError — can_transport/fakes 不存在)

- [ ] **Step 4: 写实现**

`lerobot_robot_massage/zdt/can_transport.py`:

```python
"""CAN 传输层抽象 + SocketCAN 实现 (spec §1 传输层).

SocketCanTransport 在方法内部惰性导入 python-can, 保证无 python-can
环境也能导入本模块 (驱动层单测依赖).
"""
import logging
from abc import ABC, abstractmethod
from typing import Optional

from .frames import CanFrame

logger = logging.getLogger(__name__)


class CanTransport(ABC):
    """驱动层依赖的传输抽象 (测试用 FakeTransport 注入)."""

    @abstractmethod
    def open(self) -> None:
        """打开总线 (含必要的接口配置)."""

    @abstractmethod
    def close(self) -> None:
        """关闭总线."""

    @abstractmethod
    def send(self, frame: CanFrame) -> None:
        """发送一帧."""

    @abstractmethod
    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        """阻塞接收一帧, 超时返回 None."""


class SocketCanTransport(CanTransport):
    """Linux SocketCAN (can0) 后端.

    接口需预先 up (scripts/can_setup.sh, 需 sudo):
      ip link set can0 type can bitrate 500000 && ip link set can0 up
    """

    def __init__(self, channel: str = "can0", bitrate: int = 500_000):
        self.channel = channel
        self.bitrate = bitrate
        self._bus = None

    def open(self) -> None:
        import can  # 惰性导入: 无 python-can 环境也能 import 本模块
        self._bus = can.Bus(interface="socketcan", channel=self.channel)
        logger.info("SocketCAN opened on %s", self.channel)

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def send(self, frame: CanFrame) -> None:
        import can
        if self._bus is None:
            raise RuntimeError("SocketCAN not open")
        msg = can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            data=frame.data,
        )
        self._bus.send(msg)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        import can
        if self._bus is None:
            raise RuntimeError("SocketCAN not open")
        msg = self._bus.recv(timeout=timeout_s)
        if msg is None:
            return None
        return CanFrame(arbitration_id=msg.arbitration_id, data=bytes(msg.data))
```

`lerobot_robot_massage/zdt/fakes.py`:

```python
"""测试用假对象."""
from typing import Optional

from .can_transport import CanTransport
from .frames import CanFrame


class FakeTransport(CanTransport):
    """记录发送帧 + 可注入回帧 (FIFO). recv 无注入时立即返回 None."""

    def __init__(self) -> None:
        self.sent: list[CanFrame] = []
        self.responses: list[CanFrame] = []
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.closed = True

    def send(self, frame: CanFrame) -> None:
        self.sent.append(frame)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        if self.responses:
            return self.responses.pop(0)
        return None

    def inject(self, addr: int, func: int, data_body: bytes) -> None:
        """注入一帧回帧: ID=(addr<<8), data=[func]+body."""
        self.responses.append(
            CanFrame(arbitration_id=(addr << 8), data=bytes([func]) + data_body))

    @property
    def sent_ids(self) -> list[int]:
        """已发帧 ID 列表 (断言辅助)."""
        return [f.arbitration_id for f in self.sent]
```

- [ ] **Step 5: 运行确认通过**

Run: `python lerobot_robot_massage/zdt/test_can_transport.py`
Expected: `ALL PASS`

- [ ] **Step 6: 提交**

```bash
git add lerobot_robot_massage/zdt/can_transport.py lerobot_robot_massage/zdt/testutil.py lerobot_robot_massage/zdt/fakes.py lerobot_robot_massage/zdt/test_can_transport.py
git commit -m "feat(zdt): CAN 传输抽象 + SocketCAN 惰性导入后端 + 测试辅助"
```

---

### Task 4: `zdt_driver.py` — 命令语义 + 超时重试

**Files:**
- Create: `lerobot_robot_massage/zdt/zdt_driver.py`
- Test: `lerobot_robot_massage/zdt/test_driver.py`

**Interfaces:**
- Consumes: `config.*`, `frames.*`, `can_transport.CanTransport`
- Produces:
  - `class ZdtDriverError(Exception)`, `class TimeoutError(ZdtDriverError)`, `class ChecksumError(ZdtDriverError)`
  - `class ZdtDriver(transport, timeout_s=0.1, retries=3)`:
    - `enable(addr, state: bool)` / `stop(addr)` / `stop_all()`（广播 addr=0x00）
    - `move_abs(addr, pos_deg, speed_rpm)` / `move_rel(addr, delta_deg, speed_rpm)`
    - `set_vel(addr, rpm, slope=0)`
    - `read_pos(addr) -> float`（°）
    - `read_current(addr) -> float`（mA）
    - `set_zero(addr)`（0x93）/ `home(addr)`（0x9A）
    - `on_arrived: Optional[Callable[[int],None]]` — 0xFD 到位事件回调（参数 addr）

- [ ] **Step 1: 写失败测试** `lerobot_robot_massage/zdt/test_driver.py`

```python
"""ZdtDriver 命令语义单测 (FakeTransport 注入回帧)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_ENABLE, F_POS, F_READ_POS, F_STOP
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver, TimeoutError


def _last_frame(transport: FakeTransport):
    return transport.sent[-1]


def test_enable_payload():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.enable(0x02, True)
    f = _last_frame(t)
    assert f.arbitration_id == 0x0200
    assert f.data == bytes([F_ENABLE, 0xAB, 0x01, 0x00, CHECKSUM])


def test_stop_all_broadcast():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.stop_all()
    assert _last_frame(t).arbitration_id == 0x0000      # 广播
    assert _last_frame(t).data == bytes([F_STOP, 0x98, 0x00, CHECKSUM])


def test_move_abs_payload_split():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.move_abs(0x05, 90.0, 60.0)
    # 0xFB 命令体 10 字节 → 2 帧, ID=0x0500/0x0501, 功能码 FB 重复
    assert [f.arbitration_id for f in t.sent] == [0x0500, 0x0501]
    assert t.sent[0].data[0] == F_POS
    assert t.sent[1].data[0] == F_POS
    # 位置 90.0×10=900 → 0x000384 (data[3:6])
    assert t.sent[0].data[3:6] == bytes([0x00, 0x03, 0x84])


def test_read_pos_parses():
    t = FakeTransport()
    # 回帧 [36, 符号=0x01, 位置高,中,低, 6B]; 位置 900×? → 90.0°
    t.inject(0x03, F_READ_POS, b"\x01\x00\x03\x84\x6b")
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_pos(0x03) - 90.0) < 0.001


def test_read_current_parses():
    t = FakeTransport()
    t.inject(0x03, 0x27, b"\x02\x00\x63\x6b")   # mA = 0x0063 = 99
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_current(0x03) - 99.0) < 0.001


def test_read_timeout_raises_after_retries():
    t = FakeTransport()   # 无注入 → recv 恒 None
    d = ZdtDriver(t, timeout_s=0.001, retries=2)
    try:
        d.read_pos(0x02)
        raise AssertionError("应抛 TimeoutError")
    except TimeoutError:
        pass


def test_arrival_event_dispatches_and_does_not_break_wait():
    t = FakeTransport()
    events = []
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.on_arrived = lambda addr: events.append(addr)
    # 注入: 先到位帧 (addr=0x04), 再位置回帧 (addr=0x02)
    t.inject(0x04, 0xFD, b"\x9f\x6b")
    t.inject(0x02, F_READ_POS, b"\x01\x00\x00\x64\x6b")
    pos = d.read_pos(0x02)
    assert events == [0x04]
    assert pos > 0


def test_set_zero_and_home_payload():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.set_zero(0x02)
    assert _last_frame(t).data == bytes([0x93, 0x88, 0x01, CHECKSUM])
    d.home(0x02)
    assert _last_frame(t).data == bytes([0x9A, 0x00, 0x00, CHECKSUM])


if __name__ == "__main__":
    run_all(globals())
```

> 注：`test_read_pos_parses` 的回帧布局按 ZDT 文档"地址36 符号 位置(×10) 6B"：`[36, sign, 位置3B, 6B]`。回帧字节数/符号位布局是 bring-up 核实点（spec §9），单测仅锁定当前约定。

- [ ] **Step 2: 运行确认失败**

Run: `python lerobot_robot_massage/zdt/test_driver.py`
Expected: `FAIL` (ImportError — zdt_driver 不存在)

- [ ] **Step 3: 写实现** `lerobot_robot_massage/zdt/zdt_driver.py`

```python
"""ZDT X系列V2 驱动层: 命令构造 + 响应解析 + 超时重试 (spec §2).

组帧/拆包逻辑在 frames.py; 本层负责功能码语义与错误处理.
单帧响应命令 (位置/电流/到位). 37B 参数读写为多包, bring-up 工具另行处理 (不在本层).
"""
import logging
import time
from typing import Callable, Optional

from .can_transport import CanTransport
from .config import CHECKSUM, F_ENABLE, F_POS, F_READ_CUR, F_READ_POS, F_STOP, F_VEL
from .frames import (
    add_checksum, decode_pos3, encode_frame, encode_pos3, encode_vel2,
    parse_frame, verify_checksum,
)

logger = logging.getLogger(__name__)

ArrivedCallback = Callable[[int], None]  # 参数 = 关节地址


class ZdtDriverError(Exception):
    """驱动层错误基类."""


class TimeoutError(ZdtDriverError):
    """命令超时 (重试耗尽)."""


class ChecksumError(ZdtDriverError):
    """回帧校验失败."""


class ZdtDriver:
    """向 6×ZDT 驱动发命令/读状态. 同步请求-响应, 单线程用."""

    def __init__(self, transport: CanTransport, timeout_s: float = 0.1,
                 retries: int = 3):
        self._t = transport
        self.timeout_s = timeout_s
        self.retries = retries
        self.on_arrived: Optional[ArrivedCallback] = None

    # ── 命令 (fire-and-forget 无回帧) ────────────────────

    def enable(self, addr: int, state: bool) -> None:
        body = bytes([F_ENABLE, 0xAB, 1 if state else 0, 0x00])
        self._request(addr, body, expect_response=False)

    def stop(self, addr: int) -> None:
        body = bytes([F_STOP, 0x98, 0x00])
        self._request(addr, body, expect_response=False)

    def stop_all(self) -> None:
        """广播立即停止 (addr=0x00)."""
        body = bytes([F_STOP, 0x98, 0x00])
        self._request(0x00, body, expect_response=False)

    def move_abs(self, addr: int, pos_deg: float, speed_rpm: float) -> None:
        """直通限速位置, 绝对. 位置(°)×10, 速度(RPM)×10."""
        body = (bytes([F_POS, 0x01]) + encode_vel2(speed_rpm)
                + encode_pos3(pos_deg) + b"\x0a\x00")
        self._request(addr, body, expect_response=False)

    def move_rel(self, addr: int, delta_deg: float, speed_rpm: float) -> None:
        """直通限速位置, 相对."""
        body = (bytes([F_POS, 0x00]) + encode_vel2(speed_rpm)
                + encode_pos3(delta_deg) + b"\x0a\x00")
        self._request(addr, body, expect_response=False)

    def set_vel(self, addr: int, rpm: float, slope: float = 0.0) -> None:
        """速度模式. 斜率/速度均 ×10."""
        body = (bytes([F_VEL, 0x00]) + encode_vel2(slope)
                + encode_vel2(rpm) + b"\x00")
        self._request(addr, body, expect_response=False)

    def set_zero(self, addr: int) -> None:
        """设单圈零点 (0x93 88 01, 存储)."""
        body = bytes([0x93, 0x88, 0x01])
        self._request(addr, body, expect_response=False)

    def home(self, addr: int) -> None:
        """触发回零 (0x9A 00 00)."""
        body = bytes([0x9A, 0x00, 0x00])
        self._request(addr, body, expect_response=False)

    # ── 读命令 (期待回帧) ─────────────────────────────────

    def read_pos(self, addr: int) -> float:
        """读实时位置 (度). 回帧 [36, 符号, 位置×10 3B, 6B]."""
        data = self._request(addr, bytes([F_READ_POS]), expect_response=True)
        sign = -1 if data[1] == 0x80 else 1
        return decode_pos3(data[2:5], sign)

    def read_current(self, addr: int) -> float:
        """读相电流 (mA). 回帧 [27, mA高, mA低, 6B] — bring-up 核实布局."""
        data = self._request(addr, bytes([F_READ_CUR]), expect_response=True)
        return float((data[1] << 8) | data[2])

    # ── 内部: 发送 + 同步等回帧 + 重试 ─────────────────────

    def _request(self, addr: int, body: bytes,
                 expect_response: bool) -> Optional[bytes]:
        """发送 (加校验); 期待回帧时等待并返回数据段 (含功能码), 否则 None."""
        payload = add_checksum(body)
        frames = encode_frame(addr, payload)
        for attempt in range(self.retries + 1):
            for f in frames:
                self._t.send(f)
            if not expect_response:
                return None
            resp = self._recv_for(addr, payload[0])
            if resp is not None:
                return resp
            logger.warning("timeout addr=%02X func=%02X attempt=%d/%d",
                           addr, payload[0], attempt, self.retries)
        raise TimeoutError(f"addr={addr:#04x} func={payload[0]:#04x} 超时")

    def _recv_for(self, addr: int, func: int) -> Optional[bytes]:
        """在 deadline 内收帧, 找到 (addr,func) 匹配回帧则返回, 否则 None."""
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            frame = self._t.recv(self.timeout_s)
            if frame is None:
                continue
            r_addr, _seq, data = parse_frame(frame)
            if not verify_checksum(data):
                logger.warning("checksum fail addr=%02X data=%s",
                               r_addr, data.hex())
                continue
            r_func = data[0]
            if r_func == 0xFD and self.on_arrived is not None:
                self.on_arrived(r_addr)
                continue
            if r_addr == addr and r_func == func:
                return data
        return None
```

- [ ] **Step 4: 运行确认通过**

Run: `python lerobot_robot_massage/zdt/test_driver.py`
Expected: `ALL PASS`

- [ ] **Step 5: 提交**

```bash
git add lerobot_robot_massage/zdt/zdt_driver.py lerobot_robot_massage/zdt/test_driver.py
git commit -m "feat(zdt): ZDT 驱动命令语义 + 回帧解析 + 超时重试 + 到位事件"
```

---

### Task 5: `controller.py` — ZdtController（安全层 + SerialProtocol 兼容接口）

**Files:**
- Create: `lerobot_robot_massage/zdt/controller.py`
- Test: `lerobot_robot_massage/zdt/test_controller.py`

**Interfaces:**
- Consumes: `ZdtConfig`, `CanTransport`, `ZdtDriver`
- Produces — `class ZdtController(config: ZdtConfig|None=None, transport: CanTransport|None=None)`:
  - `connect() / disconnect() / is_connected`
  - `get_state() -> (angles, vels, loads)` 各 `list[float]`（角度°、速度(占位0)、电流mA）
  - `set_joints(angles: list[float])`（clamp 限位 → 6×move_abs）
  - `set_torque(enable: bool)` / `e_stop()` / `zero()`
  - `rel_rotate(joint_id: int, delta_deg: float)`（joint_id 1-based）
  - `soft_reset()`（→ INIT_POSE_DEG）
  - `tick()`（看门狗: 超 watchdog_s 无 IO → e_stop）
  - 内部 `_last_io_s: float`（tick 判定依据）

- [ ] **Step 1: 写失败测试** `lerobot_robot_massage/zdt/test_controller.py`

```python
"""ZdtController 单测 (ZdtDriver over FakeTransport, 注入回帧)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_READ_CUR, F_READ_POS, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.testutil import run_all


def _mk(cfg=None):
    t = FakeTransport()
    ctrl = ZdtController(config=cfg or ZdtConfig(timeout_s=0.001, retries=0),
                         transport=t)
    return ctrl, t


def _inject_all_states(t: FakeTransport):
    addrs = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    for i, addr in enumerate(addrs):
        v = 1000 + i   # 位置×10 = 100.0..100.5°
        t.inject(addr, F_READ_POS, b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
    for i, addr in enumerate(addrs):
        t.inject(addr, F_READ_CUR, b"\x00" + bytes([0, 50 + i]) + b"\x6b")


def test_get_state_reads_all_joints():
    ctrl, t = _mk()
    _inject_all_states(t)
    angles, vels, loads = ctrl.get_state()
    assert len(angles) == 6
    assert abs(angles[0] - 100.0) < 0.01
    assert abs(loads[5] - 55.0) < 0.01


def test_get_state_empty_on_timeout():
    ctrl, t = _mk()   # 无注入 → read_pos 超时
    angles, vels, loads = ctrl.get_state()
    assert angles == [] and loads == []


def test_set_joints_clamps_and_sends():
    ctrl, t = _mk()
    # J1 限位 [0,360]: 送 720 → clamp 360
    ctrl.set_joints([720.0, 100.0, 0.0, 0.0, 45.0, 0.0])
    # 每轴 0xFB 2 帧 → 共 12 帧; 取 J1 的第 0/1 帧
    assert t.sent_ids[0] == 0x0200 and t.sent_ids[1] == 0x0201
    # 位置 360.0×10=3600 → 0x0E10 (data[3:6])
    assert t.sent[0].data[3:6] == bytes([0x00, 0x0E, 0x10])


def test_set_torque_sends_six_enables():
    ctrl, t = _mk()
    ctrl.set_torque(True)
    assert len(t.sent) == 6
    assert t.sent[0].data[:3] == bytes([0xF3, 0xAB, 0x01])


def test_e_stop_broadcasts():
    ctrl, t = _mk()
    ctrl.e_stop()
    assert t.sent[-1].arbitration_id == 0x0000
    assert t.sent[-1].data == bytes([0xFE, 0x98, 0x00, CHECKSUM])


def test_soft_reset_sends_init_pose():
    ctrl, t = _mk()
    ctrl.soft_reset()
    assert len(t.sent) == 12   # 6 轴 × 2 帧
    # J2 (addr 0x03) 45°×10=450 → 0x01C2, 位于其第 0 帧 data[3:6]
    j2 = [f for f in t.sent if f.arbitration_id >> 8 == 0x03]
    assert j2[0].data[3:6] == bytes([0x00, 0x01, 0xC2])


def test_rel_rotate_one_joint():
    ctrl, t = _mk()
    ctrl.rel_rotate(1, 5.0)   # joint_id 1-based
    assert t.sent[0].arbitration_id == 0x0200
    assert t.sent[0].data[1] == 0x00      # 相对标志
    assert t.sent[0].data[5] == 0x32      # 5°×10=50 → 0x32


def test_tick_triggers_estop_when_stale():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic() - 1.0   # 陈旧
    ctrl.tick()
    assert t.sent and t.sent[-1].arbitration_id == 0x0000


def test_tick_noop_when_fresh():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic()
    ctrl.tick()
    assert t.sent == []


if __name__ == "__main__":
    run_all(globals())
```

- [ ] **Step 2: 运行确认失败**

Run: `python lerobot_robot_massage/zdt/test_controller.py`
Expected: `FAIL` (ImportError — controller 不存在)

- [ ] **Step 3: 写实现** `lerobot_robot_massage/zdt/controller.py`

```python
"""ZdtController — 高层控制 + 安全层 (spec §3/§4.1).

API 与 SerialProtocol 对齐, 使 MassageRobot 可直接换协议对象 (Task 7).
安全: 位置 clamp 出口 · e_stop 广播 · 看门狗 (tick) · 电流力控 (tick 预留).
"""
import logging
import time
from typing import Optional

from .can_transport import CanTransport
from .config import INIT_POSE_DEG, ZdtConfig
from .zdt_driver import ZdtDriver, ZdtDriverError

logger = logging.getLogger(__name__)


class ZdtController:
    def __init__(self, config: Optional[ZdtConfig] = None,
                 transport: Optional[CanTransport] = None):
        self.config = config or ZdtConfig()
        self._transport = transport          # None → connect() 构造 SocketCanTransport
        self._driver: Optional[ZdtDriver] = None
        self._connected = False
        self._last_io_s = 0.0                # 看门狗依据

    # ── 连接生命周期 ─────────────────────────────────────

    def connect(self) -> None:
        if self._transport is None:
            from .can_transport import SocketCanTransport
            self._transport = SocketCanTransport(self.config.channel,
                                                 self.config.bitrate)
        self._transport.open()
        self._driver = ZdtDriver(self._transport, timeout_s=self.config.timeout_s,
                                 retries=self.config.retries)
        self.set_torque(True)
        for addr in self.config.joint_addrs:
            self._driver.read_pos(addr)      # 逐轴验证; 超时抛错 → 连接失败
        self._connected = True
        self._last_io_s = time.monotonic()
        logger.info("ZDT CAN connected: %s (6 drives verified)", self.config.channel)

    def disconnect(self) -> None:
        if self._transport is not None:
            self._transport.close()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── SerialProtocol 兼容接口 ──────────────────────────

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """角度°/速度(占位0)/电流mA. 读失败返回空列表 (调用方降级)."""
        angles: list[float] = []
        loads: list[float] = []
        try:
            for addr in self.config.joint_addrs:
                angles.append(self._driver.read_pos(addr))
            for addr in self.config.joint_addrs:
                loads.append(self._driver.read_current(addr))
        except ZdtDriverError:
            return [], [], []
        self._last_io_s = time.monotonic()
        return angles, [0.0] * len(angles), loads

    def set_joints(self, angles: list[float]) -> None:
        """clamp 到限位表 → 6×move_abs."""
        for i, addr in enumerate(self.config.joint_addrs):
            lo, hi = self.config.limits[i]
            a = max(lo, min(hi, float(angles[i])))
            self._driver.move_abs(addr, a, self.config.speed_rpm)
        self._last_io_s = time.monotonic()

    def set_torque(self, enable: bool) -> None:
        for addr in self.config.joint_addrs:
            self._driver.enable(addr, enable)
        self._last_io_s = time.monotonic()

    def e_stop(self) -> None:
        self._driver.stop_all()
        self._last_io_s = time.monotonic()
        logger.warning("EMERGENCY STOP broadcast")

    def zero(self) -> None:
        """逐轴设当前为机械零位 (0x93 88 01 存储)."""
        for addr in self.config.joint_addrs:
            self._driver.set_zero(addr)
        self._last_io_s = time.monotonic()

    # ── ZDT 扩展 ─────────────────────────────────────────

    def rel_rotate(self, joint_id: int, delta_deg: float) -> None:
        """关节相对旋转. joint_id: 1-based (1=关节1)."""
        addr = self.config.joint_addrs[joint_id - 1]
        self._driver.move_rel(addr, delta_deg, self.config.speed_rpm)
        self._last_io_s = time.monotonic()

    def soft_reset(self) -> None:
        self.set_joints(INIT_POSE_DEG)
        logger.info("soft_reset → %s", INIT_POSE_DEG)

    # ── 安全: 看门狗 + 力控 (调用方循环 tick) ─────────────

    def tick(self) -> None:
        """看门狗: >watchdog_s 无成功 IO → e_stop. 力控阈值在后续任务接入."""
        if self._connected and time.monotonic() - self._last_io_s > self.config.watchdog_s:
            logger.error("watchdog: no CAN IO for %.1fs → e_stop", self.config.watchdog_s)
            try:
                self.e_stop()
            except ZdtDriverError:
                pass
```

- [ ] **Step 4: 运行确认通过**

Run: `python lerobot_robot_massage/zdt/test_controller.py`
Expected: `ALL PASS`

- [ ] **Step 5: 提交**

```bash
git add lerobot_robot_massage/zdt/controller.py lerobot_robot_massage/zdt/test_controller.py
git commit -m "feat(zdt): ZdtController 高层控制 + 安全层 (clamp/estop/看门狗)"
```

---

### Task 6: 包导出 + 依赖 + can_setup.sh + bring-up CLI

**Files:**
- Modify: `lerobot_robot_massage/__init__.py`（补 ZdtController 轻量导出）
- Modify: `lerobot_robot_massage/pyproject.toml`（加 python-can）
- Create: `scripts/can_setup.sh`
- Create: `scripts/zdt_bringup.py`
- Test: `scripts/test_zdt_bringup_import.py`（CLI 可导入冒烟）

**Interfaces:**
- Consumes: `ZdtController`, `ZdtConfig`
- Produces:
  - `lerobot_robot_massage.zdt` 子包可从 `lerobot_robot_massage` 顶层导入
  - `scripts/can_setup.sh [iface=can0]` — 配置并 up can 接口
  - `python scripts/zdt_bringup.py status|step|reset|torque|estop`

- [ ] **Step 1: 更新 `lerobot_robot_massage/__init__.py`**

在现有轻量导入区追加（保持 try/except 风格）：

```python
# ZDT 直连 CAN 控制 (无 lerobot 依赖)
from .zdt.controller import ZdtController          # noqa: E402
from .zdt.config import ZdtConfig                  # noqa: E402
```

并更新 `__all__` 追加 `"ZdtController", "ZdtConfig"`。

- [ ] **Step 2: 更新 `pyproject.toml` dependencies**

```toml
dependencies = [
    "lerobot==0.4.4",
    "pyserial",
    "numpy",
    "opencv-python",
    "python-can[socketcan]",
]
```

- [ ] **Step 3: 写 `scripts/can_setup.sh`**

```bash
#!/bin/bash
# 配置并启用 SocketCAN 接口 (需 sudo). 用法: ./can_setup.sh [iface=can0]
set -euo pipefail
IFACE="${1:-can0}"
sudo ip link set "$IFACE" type can bitrate 500000
sudo ip link set "$IFACE" up
echo "[can_setup] $IFACE up @500k:"
ip -details link show "$IFACE" | grep -i bitrate || true
```

- [ ] **Step 4: 写 `scripts/zdt_bringup.py`**

```python
#!/usr/bin/env python3
"""ZDT 直连 CAN bring-up CLI (spec §7.1).

用法:
  python scripts/zdt_bringup.py status            # 使能+读6轴角度/电流
  python scripts/zdt_bringup.py step <j> <deg>    # 关节相对旋转 (j=1..6)
  python scripts/zdt_bringup.py reset             # soft_reset
  python scripts/zdt_bringup.py torque <0|1>
  python scripts/zdt_bringup.py estop             # 广播急停
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lerobot_robot_massage.zdt.config import INIT_POSE_DEG, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController


def _print_state(ctrl: ZdtController) -> None:
    angles, vels, loads = ctrl.get_state()
    if not angles:
        print("[status] 读取失败 (CAN 超时?)")
        return
    line = "  ".join(f"J{i+1}:{a:7.1f}° cur:{int(l):4d}mA"
                     for i, (a, l) in enumerate(zip(angles, loads)))
    print(f"[status] {line}")


def main() -> None:
    ap = argparse.ArgumentParser(description="ZDT 直连 CAN bring-up")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (默认 can0)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_status = sub.add_parser("status")
    p_status.add_argument("--n", type=int, default=1, help="读取次数")

    p_step = sub.add_parser("step")
    p_step.add_argument("joint", type=int, help="关节 1-6")
    p_step.add_argument("deg", type=float, help="相对角度")

    sub.add_parser("reset")
    p_torque = sub.add_parser("torque")
    p_torque.add_argument("state", type=int, choices=[0, 1])
    sub.add_parser("estop")
    args = ap.parse_args()

    cfg = ZdtConfig(channel=args.iface)
    ctrl = ZdtController(cfg)
    ctrl.connect()

    try:
        if args.cmd == "status":
            for _ in range(args.n):
                _print_state(ctrl)
        elif args.cmd == "step":
            if not 1 <= args.joint <= 6:
                raise SystemExit("joint 需在 1-6")
            ctrl.rel_rotate(args.joint, args.deg)
            print(f"[step] J{args.joint} {args.deg:+.1f}°")
        elif args.cmd == "reset":
            ctrl.soft_reset()
            print(f"[reset] soft_reset → {INIT_POSE_DEG}")
        elif args.cmd == "torque":
            ctrl.set_torque(bool(args.state))
            print(f"[torque] {'使能' if args.state else '失能'}")
        elif args.cmd == "estop":
            ctrl.e_stop()
            print("[estop] 已广播急停")
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 写导入冒烟测试** `scripts/test_zdt_bringup_import.py`

```python
"""zdt_bringup CLI 与顶层导出冒烟 (直接运行, 无需硬件)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from lerobot_robot_massage import ZdtConfig, ZdtController  # noqa: E402


def test_top_level_export():
    assert ZdtController is not None and ZdtConfig is not None


def test_cli_help_exits_zero():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "zdt_bringup.py"),
                        "--help"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_cli_status_parses_args():
    # 无 CAN 硬件: connect 会抛错, 但 argparse 应正确解析 (stderr 不应含 "usage" 用法错误)
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "zdt_bringup.py"),
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
```

> 注：`test_cli_status_parses_args` 在有/无硬件时都可能因连接抛错——目标是验证 argparse 正确解析、不至于因用法错误直接崩溃。无 python-can 环境时 `SocketCanTransport.open()` 抛 ImportError，同样被接受（`"usage" not in stderr`）。

- [ ] **Step 6: 运行确认**

Run: `python scripts/test_zdt_bringup_import.py`
Expected: `ALL PASS`

- [ ] **Step 7: 提交**

```bash
chmod +x scripts/can_setup.sh
git add lerobot_robot_massage/__init__.py lerobot_robot_massage/pyproject.toml scripts/can_setup.sh scripts/zdt_bringup.py scripts/test_zdt_bringup_import.py
git commit -m "feat(zdt): 顶层导出 + python-can 依赖 + can_setup + bring-up CLI"
```

---

### Task 7: MassageRobot 接入 CAN transport

**Files:**
- Modify: `lerobot_robot_massage/config_massage_robot.py`（加 transport 字段）
- Modify: `lerobot_robot_massage/massage_robot.py`（按 transport 构造协议对象）
- Test: `scripts/test_massage_robot_can.py`

**Interfaces:**
- Consumes: `MassageRobotConfig`, `ZdtController`, `SerialProtocol`
- Produces:
  - `MassageRobotConfig.transport: str = "serial"`（`"serial" | "can"`）
  - `MassageRobotConfig.channel: str = "can0"` / `can_bitrate: int = 500_000`
  - `MassageRobot.__init__` 按 transport 选择 `self._protocol = ZdtController(...)` 或 `SerialProtocol(...)`
  - `MassageRobot` 其余代码零改动（依赖同一接口）

- [ ] **Step 1: 读现状确认接口**

`config_massage_robot.py` 当前字段（port/baudrate/cameras/joint_names）——先读文件确认字段名，再追加 `transport`/`channel`/`can_bitrate`。

- [ ] **Step 2: 加配置字段**

```python
# config_massage_robot.py 内 dataclass 追加:
transport: str = "serial"       # "serial" | "can"
channel: str = "can0"           # transport=="can" 时用
can_bitrate: int = 500_000
```

- [ ] **Step 3: 改 `massage_robot.py` 构造**

```python
if config.transport == "can":
    from .zdt.config import ZdtConfig
    from .zdt.controller import ZdtController
    self._protocol = ZdtController(ZdtConfig(
        channel=config.channel, bitrate=config.can_bitrate))
else:
    self._protocol = SerialProtocol(port=config.port, baudrate=config.baudrate)
```

- [ ] **Step 4: 写冒烟测试** `scripts/test_massage_robot_can.py`

```python
"""MassageRobot CAN transport 冒烟 (无 lerobot 依赖时跳过)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from lerobot_robot_massage.config_massage_robot import MassageRobotConfig
    _HAS = True
except ImportError:
    _HAS = False


def test_config_defaults_serial():
    if not _HAS:
        return
    c = MassageRobotConfig()
    assert c.transport == "serial"


def test_config_can_fields():
    if not _HAS:
        return
    c = MassageRobotConfig(transport="can", channel="can1", can_bitrate=500_000)
    assert c.transport == "can" and c.channel == "can1"


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
```

- [ ] **Step 5: 运行确认**

Run: `python scripts/test_massage_robot_can.py`
Expected: `ALL PASS`

- [ ] **Step 6: 提交**

```bash
git add lerobot_robot_massage/config_massage_robot.py lerobot_robot_massage/massage_robot.py scripts/test_massage_robot_can.py
git commit -m "feat(zdt): MassageRobot 支持 can transport (协议对象按配置切换)"
```

---

### Task 8: 硬件 bring-up 验证（手动，需真机）

**Files:** 无代码；执行 `scripts/can_setup.sh` + `scripts/zdt_bringup.py`。

**顺序**（对应 spec §7.1，每步都先发 `e_stop` 确认急停可用）：

- [ ] **Step 1: 接线与终端电阻** — 120Ω 终端电阻在总线两端（适配器端 + 末端驱动端）；24V 供电；空载、无人。

- [ ] **Step 2: 起 can0** — `sudo bash scripts/can_setup.sh can0`；`candump can0` 确认无异常帧。

- [ ] **Step 3: 对照嗅探** — 用 STM32 串口 master 发 `rel_rotate`，`candump can0` 记录真实 0xFB 帧，与 `frames.py` 编码对比（核实位置符号位、0x0A/同步位布局）。

- [ ] **Step 4: 单轴步进** — `python scripts/zdt_bringup.py status`（应读到 6 轴角度）→ `step 1 +5` → `status` 确认 +5° 到位。

- [ ] **Step 5: 逐轴全通** — 6 轴依次 `step <j> ±N`，核对方向与到位（真机实测限位，更新 `config.DEFAULT_LIMITS`）。

- [ ] **Step 6: 急停/看门狗** — `estop` 立即停；拔适配器 → `tick()` 在 `watchdog_s` 内广播 e_stop。

- [ ] **Step 7: LeRobot 采集** — `transport="can"` 配置下跑通 `get_observation`/`send_action`（手动示教或 soft_reset 到初始位）。

- [ ] **Step 8: 修正与提交** — 若 bring-up 发现帧布局/符号约定差异，修正 `frames.py`/`zdt_driver.py` 并更新单测；提交 `fix(zdt): bring-up 修正 ...`。

---

## 后续计划（不在本计划内）

本计划交付"机械臂 PC 直连可控 + LeRobot 采集"。spec 其余子系统需独立计划：

1. **Plan 2 — 运动学与遥操**：`kinematics.py`（移植 `robot_kinematics.c` 解析式 IK + FK）→ `remote_event`/`end_event` 语义 → 控制器集成（spec §4.2-4.4）
2. **Plan 3 — 视觉遥操接入**：`ArmClient` CAN 后端（`CanArmClient`），`demo_arm_teleop.py` 零改动（spec §5）
3. **Plan 4 — 香橙派边缘部署**：可移植性核验、action-chunk 流式协议、22-DOF 扩展（spec §6）

## 自检记录

- **Spec 覆盖**：§1（分层/目录→Task 1-6）、§2（帧格式→Task 2/4）、§3（安全层→Task 5）、§4.1/§4.5（控制器+MassageRobot→Task 5/7）、§7（bring-up→Task 8）。§4.2-4.4（IK/遥操）、§5、§6 归入后续计划。
- **占位符扫描**：无 TBD/TODO。`controller.zero()` 已实装（0x93）；速度占位 0 有明确注释说明。
- **类型一致性**：`ZdtConfig.joint_addrs/limits` 在 Task 1 定义、Task 5 使用一致；`ZdtController` 接口与 `SerialProtocol` 对齐（Task 7 依赖）；`CanFrame`/`parse_frame` 签名跨 Task 2/3/4 一致；`run_all(globals())` 运行器跨所有 test 一致。
