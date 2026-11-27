"""配置与部署参数；未确认的实物参数不写死在控制代码中。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from .kinematics import KinematicsModel

FEATURES = {"groups", "calibration", "coordinate"}
PROTOCOL_LIMITS = {i: (0, 700 if i == 1 else 1000) for i in range(1, 7)}


@dataclass
class Settings:
    mode: str = "mock"
    host: str = "127.0.0.1"
    port: int = 8000
    serial_port: str = ""
    baudrate: int = 9600
    startup_delay_s: float = 5.0
    query_timeout_s: float = 1.0
    write_timeout_s: float = 1.0
    inter_command_delay_s: float = 0.03
    poll_interval_s: float = 0.25
    motion_timeout_s: float = 15.0
    position_tolerance: int = 10
    reached_samples: int = 2
    allow_hardware_writes: bool = False
    enabled_features: list[str] = field(default_factory=lambda: sorted(FEATURES))
    servo_limits: dict[int, tuple[int, int]] = field(default_factory=dict)
    kinematics: dict = field(default_factory=dict)
    coordinate_max_step_cm: float = 0.5
    coordinate_max_joint_step: int = 30
    coordinate_min_joint_delta: int | None = None
    coordinate_max_rounding_error_cm: float = 0.1
    coordinate_position_tolerance_cm: float = 0.2
    coordinate_pitch_tolerance_deg: float = 1.0
    coordinate_correction_enabled: bool = False
    coordinate_correction_max_attempts: int = 3
    coordinate_correction_max_step: int = 5
    coordinate_correction_max_offset: int = 10
    coordinate_correction_deadband: int = 1
    coordinate_correction_settle_s: float = 0.8
    coordinate_correction_stable_samples: int = 3
    coordinate_correction_stability_tolerance: int = 2
    coordinate_correction_min_progress: float = 0.1
    # 软件观察策略；位置余量不等于可控或已测量的夹持力。
    gripper_max_target_gap: int = 30
    gripper_goal_tolerance: int = 2
    gripper_min_closing_delta: int = 6
    gripper_settle_s: float = 0.5
    gripper_stable_s: float = 1.0
    gripper_stable_samples: int = 4
    gripper_stability_tolerance: int = 2
    gripper_observation_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        if self.mode not in {"mock", "serial"}:
            raise ValueError("mode 必须为 mock 或 serial")
        if self.host != "127.0.0.1":
            raise ValueError("第一版本机服务仅监听 127.0.0.1")
        for name, low, high in [("port", 1, 65535), ("baudrate", 9600, 9600),
                                 ("position_tolerance", 0, 100), ("reached_samples", 1, 20),
                                 ("coordinate_max_joint_step", 1, 100),
                                 ("coordinate_correction_max_attempts", 1, 10),
                                 ("coordinate_correction_max_step", 1, 10),
                                 ("coordinate_correction_max_offset", 1, 30),
                                 ("coordinate_correction_deadband", 0, 10),
                                 ("coordinate_correction_stable_samples", 2, 10),
                                 ("coordinate_correction_stability_tolerance", 0, 5),
                                 ("gripper_max_target_gap", 1, 100),
                                 ("gripper_goal_tolerance", 0, 20),
                                 ("gripper_min_closing_delta", 1, 100),
                                 ("gripper_stable_samples", 2, 20),
                                 ("gripper_stability_tolerance", 0, 20)]:
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} 必须为 {low}～{high} 的整数")
        if (self.coordinate_min_joint_delta is not None
                and (type(self.coordinate_min_joint_delta) is not int
                     or not 1 <= self.coordinate_min_joint_delta <= 100)):
            raise ValueError("coordinate_min_joint_delta 必须为 1～100 的整数或 null")
        for name in ("query_timeout_s", "write_timeout_s", "poll_interval_s", "motion_timeout_s"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 3600:
                raise ValueError(f"{name} 必须为有限正数，最多 3600 秒")
        if (type(self.startup_delay_s) not in (int, float)
                or not math.isfinite(self.startup_delay_s)
                or not 0 <= self.startup_delay_s <= 3600):
            raise ValueError("startup_delay_s 必须为有限非负数，最多 3600 秒")
        if self.poll_interval_s < 0.05:
            raise ValueError("poll_interval_s 不得小于 0.05")
        if type(self.inter_command_delay_s) not in (int, float) or not math.isfinite(self.inter_command_delay_s) or not 0.01 <= self.inter_command_delay_s <= 1:
            raise ValueError("inter_command_delay_s 必须在 0.01～1 秒之间")
        if type(self.allow_hardware_writes) is not bool:
            raise ValueError("allow_hardware_writes 必须为布尔值")
        if type(self.coordinate_correction_enabled) is not bool:
            raise ValueError("coordinate_correction_enabled 必须为布尔值")
        if type(self.serial_port) is not str:
            raise ValueError("serial_port 必须为字符串")
        if self.mode == "serial" and not self.serial_port.strip():
            raise ValueError("串口模式需要明确配置 serial_port")
        if type(self.enabled_features) is not list or any(type(v) is not str or v not in FEATURES for v in self.enabled_features):
            raise ValueError("enabled_features 只能包含 groups、calibration、coordinate")
        for name, low, high in (("coordinate_max_step_cm", 0.1, 5.0),
                                ("coordinate_max_rounding_error_cm", 0.001, 1.0),
                                ("coordinate_position_tolerance_cm", 0.01, 2.0),
                                ("coordinate_pitch_tolerance_deg", 0.1, 10.0),
                                ("coordinate_correction_settle_s", 0.2, 10.0),
                                ("coordinate_correction_min_progress", 0.01, 10.0),
                                ("gripper_settle_s", 0.0, 10.0),
                                ("gripper_stable_s", 0.1, 10.0),
                                ("gripper_observation_timeout_s", 0.2, 30.0)):
            value = getattr(self, name)
            if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
                raise ValueError(f"{name} 必须为 {low}～{high} 范围内的有限数值")
        if self.gripper_min_closing_delta <= self.gripper_stability_tolerance:
            raise ValueError("gripper_min_closing_delta 必须大于 gripper_stability_tolerance")
        if self.gripper_max_target_gap <= self.gripper_goal_tolerance:
            raise ValueError("gripper_max_target_gap 必须大于 gripper_goal_tolerance")
        # 采样间隔的可行性在实际闭爪前检查；不能因新增功能拒绝旧的慢轮询配置。
        gripper_minimum_observation = self.gripper_settle_s + self.gripper_stable_s
        if self.gripper_observation_timeout_s <= gripper_minimum_observation:
            raise ValueError("gripper_observation_timeout_s 必须大于等待及稳定观察所需时间")
        if type(self.kinematics) is not dict:
            raise ValueError("kinematics 必须为模型参数对象")
        self.kinematics = KinematicsModel.from_config(self.kinematics).to_config()
        if type(self.servo_limits) is not dict:
            raise ValueError("servo_limits 必须为对象")
        normalized = {}
        for key, value in self.servo_limits.items():
            if type(key) is int:
                servo_id = key
            elif type(key) is str and key in {str(i) for i in range(1, 7)}:
                servo_id = int(key)
            else:
                raise ValueError("servo_limits 的 ID 必须为 1～6")
            if servo_id not in PROTOCOL_LIMITS or servo_id in normalized or type(value) not in (list, tuple) or len(value) != 2:
                raise ValueError("servo_limits 必须使用唯一 ID 和 [最小值, 最大值]")
            low, high = value
            if type(low) is not int or type(high) is not int or not 0 <= low <= high <= PROTOCOL_LIMITS[servo_id][1]:
                raise ValueError("servo_limits 超出总线协议允许范围")
            normalized[servo_id] = (low, high)
        self.servo_limits = normalized
        if self.mode == "serial" and self.allow_hardware_writes and set(normalized) != set(range(1, 7)):
            raise ValueError("启用实物写命令前，需配置全部六个关节的实际有效范围")

    @classmethod
    def load(cls, path: str | Path) -> Settings:
        def unique_pairs(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError(f"配置中存在重复字段：{key}")
                result[key] = value
            return result

        data = json.loads(Path(path).read_text(encoding="utf-8-sig"), object_pairs_hook=unique_pairs)
        if not isinstance(data, dict):
            raise ValueError("配置文件顶层必须为 JSON 对象")
        return cls(**data)

    def limits(self) -> dict[int, tuple[int, int]]:
        if self.mode == "mock":
            return {**PROTOCOL_LIMITS, **self.servo_limits}
        return dict(self.servo_limits)

    def coordinate_min_delta(self) -> int:
        """Inclusive coordinate motion threshold; independent when configured.

        Omitted/null retains the previous strict ``delta > position_tolerance``
        precheck. This threshold never changes joint or model acceptance.
        """
        if self.coordinate_min_joint_delta is None:
            return self.position_tolerance + 1
        return self.coordinate_min_joint_delta
