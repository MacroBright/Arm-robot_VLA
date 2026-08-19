"""Regression tests for safe MuJoCo startup on shared servers."""

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
        str(Path(mujoco_sim.__file__).resolve().parent / "scene.xml"),
        enable_frame_buffers=False,
    )
    assert arm._shm is None
    assert arm._shm_ee is None
    arm.cleanup()
