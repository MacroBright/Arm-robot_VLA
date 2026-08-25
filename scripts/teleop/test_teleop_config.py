"""scripts/teleop/test_teleop_config.py — 集中配置文件单元测试."""
import tempfile
from pathlib import Path
import pytest

from scripts.teleop.teleop_config import (
    DEFAULT_TELEOP_CONFIG,
    TeleopConfig,
    GearConfig,
    JointFactorConfig,
    MotorLimitConfig,
    PresetPoseConfig,
    VisionFilterConfig,
)


def test_default_teleop_config_structure():
    cfg = DEFAULT_TELEOP_CONFIG
    assert cfg.gear.default_gear == 2
    assert cfg.gear.gear_1_low.lin_scale == 0.030
    assert cfg.gear.gear_2_mid.lin_scale == 0.050
    assert cfg.gear.gear_3_high.lin_scale == 0.070

    factors = cfg.joint_factor.as_list()
    assert len(factors) == 6
    assert factors == [2.0, 2.0, 2.0, 2.0, 1.0, 2.0]

    assert cfg.motor.speed_rpm == 2800.0
    assert cfg.motor.position_acc == 0
    assert cfg.pose.ready_pose_deg == [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]
    assert cfg.vision.deadband_angle_deg == 5.0


def test_teleop_config_to_dict_and_from_dict():
    cfg = TeleopConfig()
    cfg.gear.gear_2_mid.lin_scale = 0.08
    cfg.joint_factor.j1_base_yaw = 2.5
    d = cfg.to_dict()
    assert isinstance(d, dict)
    assert d["gear"]["gear_2_mid"]["lin_scale"] == 0.08

    restored = TeleopConfig.from_dict(d)
    assert restored.gear.gear_2_mid.lin_scale == 0.08
    assert restored.joint_factor.j1_base_yaw == 2.5


def test_teleop_config_load_yaml():
    yaml_path = Path(__file__).parent / "teleop_config.yaml"
    if yaml_path.exists():
        cfg = TeleopConfig.load(yaml_path)
        assert cfg.gear.default_gear == 2
        assert cfg.joint_factor.j5_wrist_pitch == 1.0
        assert cfg.joint_factor.j1_base_yaw == 2.0
        assert cfg.motor.speed_rpm == 2800.0


def test_teleop_config_save_and_load_temp():
    cfg = TeleopConfig()
    cfg.gear.default_gear = 3
    cfg.motor.speed_rpm = 2500.0
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_yaml = Path(tmp_dir) / "test_cfg.yaml"
        cfg.save_yaml(tmp_yaml)
        assert tmp_yaml.exists()

        loaded = TeleopConfig.load(tmp_yaml)
        assert loaded.gear.default_gear == 3
        assert loaded.motor.speed_rpm == 2500.0
