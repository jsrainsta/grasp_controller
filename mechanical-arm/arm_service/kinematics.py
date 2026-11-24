"""Nominal LeArm bus-servo kinematics; no serial I/O or physical calibration claims.

The base frame is right handed: +X is the zero-yaw direction and +Z is up.
The tool point lies on the pitch chain's axis at ``tool_length_cm``. FK is an
estimate from joint feedback, not an independently measured Cartesian pose.
"""

from __future__ import annotations

import math


class KinematicsError(ValueError):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = dict(details or {})


_JOINTS = (3, 4, 5, 6)
_DEFAULTS = {
    "base_height_cm": 2.89,
    "upper_arm_cm": 10.43,
    "forearm_cm": 8.9,
    "tool_length_cm": 17.7,
    "units_per_degree": 1000 / 240,
}
_ANGLE_LIMITS = {3: (-90.0, 90.0), 4: (-90.0, 90.0),
                 5: (0.0, 180.0), 6: (-90.0, 90.0)}


def _number(value, name: str, *, positive: bool = False) -> float:
    if type(value) not in (int, float):
        raise KinematicsError("coordinate_pose_invalid", f"{name} 必须为有限数值")
    try:
        valid = math.isfinite(value) and (not positive or value > 0)
        result = float(value)
    except (OverflowError, ValueError):
        valid, result = False, 0.0
    if not valid:
        raise KinematicsError("coordinate_pose_invalid", f"{name} 必须为有限" + ("正数" if positive else "数值"))
    return result


class KinematicsModel:
    """Four positioning joints (3..6); claw 1 and tool roll 2 stay untouched."""

    def __init__(self, config: dict | None = None):
        data = {} if config is None else config
        if type(data) is not dict or set(data) - (set(_DEFAULTS) | {"zero_positions"}):
            raise KinematicsError("coordinate_pose_invalid", "运动学配置必须为对象且不能包含未知字段")
        for name, default in _DEFAULTS.items():
            value = _number(data.get(name, default), name, positive=name != "base_height_cm")
            if name == "base_height_cm" and value < 0:
                raise KinematicsError("coordinate_pose_invalid", "base_height_cm 不得小于零")
            setattr(self, name, value)
        zeros = data.get("zero_positions", {})
        if type(zeros) is not dict:
            raise KinematicsError("coordinate_pose_invalid", "zero_positions 必须为对象")
        normalized = {str(i): 500.0 for i in _JOINTS}
        seen = set()
        for key, value in zeros.items():
            if type(key) is int and key in _JOINTS:
                key = str(key)
            elif type(key) is not str or key not in normalized:
                raise KinematicsError("coordinate_pose_invalid", "zero_positions 只能包含 3、4、5、6")
            if key in seen:
                raise KinematicsError("coordinate_pose_invalid", "zero_positions 中关节编号重复")
            seen.add(key)
            position = _number(value, f"zero_positions.{key}")
            if not 0 <= position <= 1000:
                raise KinematicsError("coordinate_pose_invalid", "zero_positions 必须处于 0～1000")
            normalized[key] = position
        self.zero_positions = normalized

    @classmethod
    def from_config(cls, data: dict) -> KinematicsModel:
        if type(data) is not dict:
            raise KinematicsError("coordinate_pose_invalid", "运动学配置必须为对象")
        return cls(data)

    def to_config(self) -> dict:
        return {**{name: getattr(self, name) for name in _DEFAULTS},
                "zero_positions": dict(self.zero_positions)}

    def _angles(self, positions: dict[int, int]) -> dict[int, float]:
        if type(positions) is not dict:
            raise KinematicsError("coordinate_pose_invalid", "关节反馈必须为对象")
        for key, value in positions.items():
            # Position replies use 0..1000 for every ID. The claw's 700 cap
            # applies to outgoing targets, not to unrelated readback here.
            if type(key) is not int or not 1 <= key <= 6 or type(value) is not int or not 0 <= value <= 1000:
                raise KinematicsError("coordinate_pose_invalid", "关节反馈必须使用有效的整数编号和原始位置")
        missing = set(_JOINTS) - set(positions)
        if missing:
            raise KinematicsError("coordinate_pose_invalid", "缺少定位关节反馈", {"missing_ids": sorted(missing)})
        angles = {i: (positions[i] - self.zero_positions[str(i)]) / self.units_per_degree
                  for i in _JOINTS}
        angles[5] = 90 - angles[5]
        if not all(math.isfinite(value) for value in angles.values()):
            raise KinematicsError("coordinate_pose_invalid", "关节角计算结果非有限")
        return angles

    def forward(self, positions: dict[int, int]) -> dict:
        angles = self._angles(positions)
        yaw, shoulder, elbow, wrist = (math.radians(angles[i]) for i in (6, 5, 4, 3))
        pitch = shoulder + elbow + wrist
        radial = (self.upper_arm_cm * math.cos(shoulder)
                  + self.forearm_cm * math.cos(shoulder + elbow)
                  + self.tool_length_cm * math.cos(pitch))
        z = (self.base_height_cm + self.upper_arm_cm * math.sin(shoulder)
             + self.forearm_cm * math.sin(shoulder + elbow)
             + self.tool_length_cm * math.sin(pitch))
        pose = {"x_cm": radial * math.cos(yaw), "y_cm": radial * math.sin(yaw),
                "z_cm": z, "pitch_deg": angles[5] + angles[4] + angles[3],
                "frame": "base_link", "estimated": True}
        if not all(math.isfinite(pose[name]) for name in ("x_cm", "y_cm", "z_cm", "pitch_deg")):
            raise KinematicsError("coordinate_pose_invalid", "运动学模型产生非有限坐标")
        return pose

    def check_nominal(self, positions: dict[int, int]) -> None:
        """Reject a raw joint configuration outside the model's angular range.

        These are model limits, not a physical swept-collision check.
        """
        for servo_id, angle in self._angles(positions).items():
            low, high = _ANGLE_LIMITS[servo_id]
            if not low - 1e-9 <= angle <= high + 1e-9:
                raise KinematicsError("coordinate_pose_invalid", "当前位置超出模型名义角度范围",
                                      {"servo_id": servo_id, "angle_deg": angle,
                                       "nominal_range_deg": [low, high]})

    @staticmethod
    def _valid_limits(limits: dict) -> dict[int, tuple[int, int]]:
        if type(limits) is not dict:
            raise KinematicsError("limits_missing", "必须配置定位关节范围")
        missing = set(_JOINTS) - set(limits)
        if missing:
            raise KinematicsError("limits_missing", "缺少定位关节范围", {"missing_ids": sorted(missing)})
        result = {}
        for key, bounds in limits.items():
            if (type(key) is not int or not 1 <= key <= 6
                    or type(bounds) not in (tuple, list) or len(bounds) != 2
                    or any(type(value) is not int for value in bounds)
                    or not 0 <= bounds[0] <= bounds[1] <= (700 if key == 1 else 1000)):
                raise KinematicsError("coordinate_pose_invalid", "关节范围必须使用整数编号及有效 [min,max]")
            result[key] = tuple(bounds)
        return result

    def plan_increment(self, positions: dict[int, int], delta_cm: tuple,
                       limits: dict[int, tuple[int, int]], *, max_step_cm: float = 0.5,
                       max_joint_step: int = 30, max_rounding_error_cm: float = 0.1) -> dict:
        """Solve, quantize and validate one target; does not check swept collisions.

        Both radial signs and elbow branches are examined. The nearest feasible
        joint solution is used, preventing an arbitrary branch flip. The raw
        feedback may lie just outside configured target limits; only targets
        must be inside them, as real feedback has finite resolution/deadband.
        """
        max_step = _number(max_step_cm, "max_step_cm", positive=True)
        max_jump = _number(max_joint_step, "max_joint_step", positive=True)
        max_error = _number(max_rounding_error_cm, "max_rounding_error_cm", positive=True)
        if type(delta_cm) not in (tuple, list) or len(delta_cm) != 3:
            raise KinematicsError("coordinate_pose_invalid", "delta_cm 必须包含三个有限数值")
        delta = tuple(_number(value, "delta_cm") for value in delta_cm)
        distance = math.hypot(*delta)
        if not math.isfinite(distance):
            raise KinematicsError("coordinate_pose_invalid", "坐标增量的数值尺度无效")
        if distance > max_step + 1e-12:
            raise KinematicsError("coordinate_step_too_large", "坐标增量超过单次最大距离",
                                  {"distance_cm": distance, "max_step_cm": max_step})
        self.check_nominal(positions)
        normalized_limits = self._valid_limits(limits)
        start = self.forward(positions)
        requested = dict(start)
        for name, change in zip(("x_cm", "y_cm", "z_cm"), delta):
            requested[name] += change
        x, y, z = (requested[name] for name in ("x_cm", "y_cm", "z_cm"))
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise KinematicsError("coordinate_pose_invalid", "目标坐标非有限")
        radius = math.hypot(x, y)
        if radius < 1e-9:
            raise KinematicsError("coordinate_singularity", "目标位于底座旋转轴，无法可靠确定底座角度")
        pitch = math.radians(start["pitch_deg"])
        a, b = self.upper_arm_cm, self.forearm_cm
        nominal, singular = [], False
        for radial in (radius, -radius):
            raw_yaw = math.degrees(math.atan2(y, x)) + (180 if radial < 0 else 0)
            yaw = (raw_yaw + 180) % 360 - 180
            if not -90 - 1e-9 <= yaw <= 90 + 1e-9:
                continue
            u = radial - self.tool_length_cm * math.cos(pitch)
            v = z - self.base_height_cm - self.tool_length_cm * math.sin(pitch)
            try:
                cosine = (u * u + v * v - a * a - b * b) / (2 * a * b)
            except (OverflowError, ZeroDivisionError):
                raise KinematicsError("coordinate_pose_invalid", "运动学参数的数值尺度无效") from None
            if not math.isfinite(cosine):
                raise KinematicsError("coordinate_pose_invalid", "逆解计算产生非有限数值")
            if cosine < -1 - 1e-10 or cosine > 1 + 1e-10:
                continue
            elbow_magnitude = math.acos(max(-1.0, min(1.0, cosine)))
            if abs(math.sin(elbow_magnitude)) < 1e-7:
                singular = True
                continue
            for elbow in (elbow_magnitude, -elbow_magnitude):
                shoulder_base = math.atan2(v, u) - math.atan2(b * math.sin(elbow), a + b * math.cos(elbow))
                for turn in (-1, 0, 1):
                    shoulder = math.degrees(shoulder_base) + 360 * turn
                    candidate_angles = {3: start["pitch_deg"] - shoulder - math.degrees(elbow),
                                        4: math.degrees(elbow), 5: shoulder, 6: yaw}
                    if any(not low - 1e-8 <= candidate_angles[i] <= high + 1e-8
                           for i, (low, high) in _ANGLE_LIMITS.items()):
                        continue
                    raw_target = {i: self.zero_positions[str(i)] + self.units_per_degree *
                                  (90 - candidate_angles[i] if i == 5 else candidate_angles[i])
                                  for i in _JOINTS}
                    if not all(math.isfinite(value) for value in raw_target.values()):
                        raise KinematicsError("coordinate_pose_invalid", "关节映射参数的数值尺度无效")
                    target = {i: round(value) for i, value in raw_target.items()}
                    if any(not 0 <= value <= 1000 for value in target.values()):
                        continue
                    # Rounding must not cross a nominal model boundary.
                    rounded_angles = self._angles(target)
                    if any(not low - 1e-9 <= rounded_angles[i] <= high + 1e-9
                           for i, (low, high) in _ANGLE_LIMITS.items()):
                        continue
                    if target not in nominal:
                        nominal.append(target)
        if not nominal:
            code = "coordinate_singularity" if singular else "coordinate_unreachable"
            raise KinematicsError(code, "目标处于运动学奇异位置" if singular else "目标在模型名义范围内无可用逆解")
        bounded = [target for target in nominal
                   if all(normalized_limits[i][0] <= target[i] <= normalized_limits[i][1] for i in _JOINTS)]
        if not bounded:
            candidates = []
            for target in nominal:
                violations = [{"servo_id": i, "position": target[i],
                               "min": normalized_limits[i][0], "max": normalized_limits[i][1]}
                              for i in _JOINTS
                              if not normalized_limits[i][0] <= target[i] <= normalized_limits[i][1]]
                candidates.append({"targets": dict(target), "violations": violations})
            raise KinematicsError("coordinate_limits_violation", "逆解目标超出已配置的关节范围",
                                  {"start_positions": dict(positions), "requested_pose": requested,
                                   "limits": {i: list(normalized_limits[i]) for i in _JOINTS},
                                   "candidates": candidates})
        nearby = [target for target in bounded if max(abs(target[i] - positions[i]) for i in _JOINTS) <= max_jump]
        if not nearby:
            smallest = min(max(abs(target[i] - positions[i]) for i in _JOINTS) for target in bounded)
            raise KinematicsError("coordinate_joint_step_too_large", "逆解所需关节变化超过单次限制",
                                  {"required_joint_step": smallest, "max_joint_step": max_jump})
        candidates = []
        best_error = math.inf
        for target in nearby:
            predicted = self.forward(target)
            error = math.hypot(*(predicted[name] - requested[name] for name in ("x_cm", "y_cm", "z_cm")))
            best_error = min(best_error, error)
            if error <= max_error + 1e-12:
                changes = {i: target[i] - positions[i] for i in _JOINTS}
                candidates.append((sum(change * change for change in changes.values()), error,
                                   target, predicted, changes))
        if not candidates:
            raise KinematicsError("coordinate_rounding_error", "原始位置量化后的坐标误差超过限制",
                                  {"rounding_error_cm": best_error, "max_rounding_error_cm": max_error})
        _, error, target, predicted, changes = min(candidates, key=lambda entry: (entry[0], entry[1]))
        if not any(changes.values()):
            raise KinematicsError("coordinate_resolution_limit", "增量小于当前关节位置分辨率，未生成新的目标")
        return {"backend": "host_ik", "frame": "base_link", "estimated": True,
                "pitch_policy": "keep_current", "start_pose": start,
                "requested_pose": requested, "predicted_pose": predicted,
                "delta_cm": list(delta),
                "predicted_delta_cm": [predicted[name] - start[name] for name in ("x_cm", "y_cm", "z_cm")],
                "targets": target, "joint_deltas": changes,
                "rounding_error_cm": error, "max_joint_delta": max(abs(change) for change in changes.values())}
