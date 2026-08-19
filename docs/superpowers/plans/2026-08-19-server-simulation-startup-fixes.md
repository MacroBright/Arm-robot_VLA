# Server Simulation Startup Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the MuJoCo simulator start with its tracked default scene and avoid all shared-memory access when `--no-camera` is selected.

**Architecture:** Keep the camera-enabled inter-process contract unchanged. Add a constructor switch that disables frame-buffer allocation, have the CLI map `--no-camera` to that switch, and make cleanup explicitly tolerate disabled buffers.

**Tech Stack:** Python 3.10, MuJoCo 3.10, pytest 8.4, multiprocessing.shared_memory

---

### Task 1: Add failing startup regression tests

**Files:**
- Create: `scripts/simulation/test_mujoco_startup.py`
- Test: `scripts/simulation/test_mujoco_startup.py`

- [ ] **Step 1: Write tests for the tracked default scene and disabled buffers**

```python
from pathlib import Path

import mujoco_sim


def test_default_scene_path_is_tracked_scene():
    expected = Path(mujoco_sim.__file__).resolve().parent / "scene.xml"
    assert mujoco_sim.SCENE_PATH == expected
    assert mujoco_sim.SCENE_PATH.is_file()


def test_disabled_frame_buffers_never_open_shared_memory(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("SharedMemory must not be used with frame buffers disabled")

    monkeypatch.setattr(mujoco_sim.shared_memory, "SharedMemory", fail_if_called)
    arm = mujoco_sim.MuJoCoArm(
        str(mujoco_sim.SCENE_PATH), enable_frame_buffers=False
    )
    assert arm._shm is None
    assert arm._shm_ee is None
    arm.cleanup()
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
PYTHONPATH="$PWD" python -m pytest -q scripts/simulation/test_mujoco_startup.py
```

Expected: both tests fail—one because `SCENE_PATH` uses `mujoco_scene/scene.xml`, and one because `MuJoCoArm` does not accept `enable_frame_buffers`.

- [ ] **Step 3: Commit the failing tests**

```bash
git add scripts/simulation/test_mujoco_startup.py
git commit -m "test: cover headless simulation startup"
```

### Task 2: Implement the minimum startup fixes

**Files:**
- Modify: `scripts/simulation/mujoco_sim.py`
- Test: `scripts/simulation/test_mujoco_startup.py`

- [ ] **Step 1: Point `SCENE_PATH` at the tracked scene**

Replace the constant with:

```python
SCENE_PATH = Path(__file__).resolve().parent / "scene.xml"
```

- [ ] **Step 2: Make frame buffers optional**

Add `enable_frame_buffers: bool = True` to `MuJoCoArm.__init__`, initialize `_shm` and `_shm_ee` to `None`, and execute the existing open/unlink/create block only when the flag is true. Update `cleanup()` to skip `None` values.

- [ ] **Step 3: Wire `--no-camera` to the constructor**

Construct the arm as:

```python
arm = MuJoCoArm(
    str(scene_path),
    use_ik=args.ik,
    enable_frame_buffers=not args.no_camera,
)
```

- [ ] **Step 4: Run focused tests and verify GREEN**

```bash
PYTHONPATH="$PWD" python -m pytest -q scripts/simulation/test_mujoco_startup.py
```

Expected: `2 passed`.

- [ ] **Step 5: Commit the implementation**

```bash
git add scripts/simulation/mujoco_sim.py
git commit -m "fix: make headless simulation safe on shared servers"
```

### Task 3: Document and verify the improvement

**Files:**
- Modify: `docs/MUJOCO_SIM.md`

- [ ] **Step 1: Document server-safe no-camera behavior**

State that `--no-camera` disables the renderer and both shared-memory frame buffers, making it appropriate for TCP/physics smoke tests on shared servers.

- [ ] **Step 2: Run regression verification**

Run the focused tests, the existing 74-test no-hardware suite, `python -m pip check`, and a temporary `mujoco_sim.py --no-viewer --no-camera --port 15555` TCP smoke test using the default scene. Verify the port closes and no `maziqi` MuJoCo shared-memory objects remain.

- [ ] **Step 3: Commit documentation and test evidence**

```bash
git add docs/MUJOCO_SIM.md
git commit -m "docs: explain server-safe simulation mode"
```
