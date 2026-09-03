"""EpisodeRecorder 测试 — JSONL schema + observation/action 分离 + 帧文件引用 (spec TASK-35)."""
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402


def _out(tag):
    p = Path("/tmp") / f"rec_{tag}"
    if p.exists():
        shutil.rmtree(p)
    return p


def _obs(q=(0.0,) * 6):
    return {
        "q": list(q),
        "dq": [0.0] * 6,
        "current": [10.0] * 6,
        "ee_pose": {"position": [0.0, 0.0, 0.0],
                    "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "hand_pose": {"position": [0.0, 0.0, 100.0],
                      "orientation": [0.0, 0.0, 0.0], "confidence": 0.9},
    }


def _act():
    return {"cartesian_command": {"linear_velocity": [1.0, 0.0, 0.0],
                                  "angular_velocity": [0.0, 0.0, 0.0],
                                  "timestamp": 0.0},
            "commanded_joint_target": [0.0] * 6}


def _safety():
    return {"phase": "TELEOP", "sigma_min": 0.1, "condition": 5.0, "workspace_ok": True}


def test_add_record_writes_valid_jsonl_schema():
    out = _out("a")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety(), camera_ts=1.5)
    rec.finish_episode()
    lines = (out / ep / "data.jsonl").read_text().strip().splitlines()
    assert len(lines) == 1
    row = json.loads(lines[0])
    assert set(row) == {"timestamp", "observation", "action", "safety"}
    assert set(row["observation"]) == {"q", "dq", "current", "ee_pose", "hand_pose"}
    assert set(row["action"]) == {"cartesian_command", "commanded_joint_target"}
    assert row["safety"]["phase"] == "TELEOP"
    # observation / action 分离 (spec §7)
    assert row["observation"]["q"][0] == 0.0
    assert row["action"]["cartesian_command"]["linear_velocity"] == [1.0, 0.0, 0.0]


def test_records_camera_frames_files():
    out = _out("b")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    color = (np.random.rand(4, 4, 3) * 255).astype("uint8")
    depth = (np.random.rand(4, 4) * 1000).astype("uint16")
    rec.add_record(_obs(), _act(), _safety(), color=color, depth=depth, camera_ts=2.0)
    rec.finish_episode()
    row = json.loads((out / ep / "data.jsonl").read_text().strip().splitlines()[0])
    frames = row["observation"]["camera_frames"]
    assert set(frames) == {"color", "depth"}
    assert row["observation"]["camera_ts"] == 2.0
    # 文件真实存在 (修订 #5: 引用帧文件, 非仅时间戳)
    assert (out / ep / frames["color"]).exists()
    assert (out / ep / frames["depth"]).exists()
    assert not frames["color"].startswith(str(ep))     # 相对路径


def test_no_frames_when_save_frames_false():
    out = _out("c")
    rec = EpisodeRecorder(out, save_frames=False)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety(), color=np.zeros((2, 2, 3), "uint8"))
    rec.finish_episode()
    row = json.loads((out / ep / "data.jsonl").read_text().strip().splitlines()[0])
    assert "camera_frames" not in row["observation"]
    assert not (out / ep / "frames").exists()


def test_multiple_episodes_separate_dirs():
    out = _out("d")
    rec = EpisodeRecorder(out)
    ep1 = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    rec.finish_episode()
    ep2 = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    rec.finish_episode()
    assert ep1 != ep2
    assert (out / ep1 / "data.jsonl").exists()
    assert (out / ep2 / "data.jsonl").exists()


def test_finish_returns_stats():
    out = _out("e")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    stats = rec.finish_episode()
    assert stats["records"] == 1
    assert stats["path"] == str(out / ep)


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
