"""Pure feedback assessment and bounded joint-command correction planning.

No I/O, retries or motion happen here. Model acceptance compares feedback FK
with one fixed requested pose; it is not independent Cartesian measurement.
"""

from __future__ import annotations

import math

from .kinematics import KinematicsError, KinematicsModel


_JOINTS = (3, 4, 5, 6)
_POSE_KEYS = ("x_cm", "y_cm", "z_cm", "pitch_deg")


class CorrectionError(ValueError):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = dict(details or {})


def _integer(value, name: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} 必须为不小于 {minimum} 的整数")
    return value


def _positive(value, name: str) -> float:
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value > 0
    except (OverflowError, ValueError):
        valid = False
    if not valid:
        raise ValueError(f"{name} 必须为有限正数")
    return float(value)


def _positions(value, name: str, *, only_controlled: bool = False) -> dict[int, int]:
    if type(value) is not dict:
        raise ValueError(f"{name} 必须为关节位置对象")
    result = {}
    allowed = _JOINTS if only_controlled else range(1, 7)
    for key, position in value.items():
        if type(key) is str and key in {str(i) for i in allowed}:
            servo_id = int(key)
        elif type(key) is int and key in allowed:
            servo_id = key
        else:
            raise ValueError(f"{name} 的关节编号无效")
        if servo_id in result or type(position) is not int or not 0 <= position <= 1000:
            raise ValueError(f"{name} 需要唯一关节编号和 0～1000 的整数位置")
        result[servo_id] = position
    if set(_JOINTS) - set(result):
        raise ValueError(f"{name} 缺少 3、4、5、6 号定位关节")
    return result


def evaluate_feedback(model: KinematicsModel, positions: dict[int, int], plan: dict, *,
                      joint_tolerance: int, position_tolerance_cm: float,
                      pitch_tolerance_deg: float, observed_at: str | None = None) -> dict:
    """Assess one sample against the original plan, never a shifted goal."""
    try:
        joint_tol = _integer(joint_tolerance, "joint_tolerance")
        position_tol = _positive(position_tolerance_cm, "position_tolerance_cm")
        pitch_tol = _positive(pitch_tolerance_deg, "pitch_tolerance_deg")
        if observed_at is not None and type(observed_at) is not str:
            raise ValueError("observed_at 必须为字符串或 None")
        feedback = _positions(positions, "positions")
        if type(plan) is not dict or type(plan.get("requested_pose")) is not dict:
            raise ValueError("plan 必须包含固定的 requested_pose")
        requested = plan["requested_pose"]
        for key in _POSE_KEYS:
            value = requested.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"requested_pose.{key} 必须为有限数值")
        if requested.get("frame", "base_link") != "base_link" or plan.get("frame", "base_link") != "base_link":
            raise ValueError("反馈验收仅支持 base_link 坐标系")
        desired = _positions(plan.get("targets"), "plan.targets", only_controlled=True)
        observed = model.forward(feedback)
        errors = {i: feedback[i] - desired[i] for i in _JOINTS}
        vector = [observed[key] - requested[key] for key in _POSE_KEYS[:3]]
        position_error = math.hypot(*vector)
        pitch_error = observed["pitch_deg"] - requested["pitch_deg"]
        score = max(position_error / position_tol, abs(pitch_error) / pitch_tol,
                    max(abs(error) for error in errors.values()) / max(1, joint_tol))
        if not all(math.isfinite(value) for value in (*vector, position_error, pitch_error, score)):
            raise ValueError("反馈验收的数值尺度无效")
    except (ValueError, OverflowError, TypeError) as error:
        if isinstance(error, KinematicsError):
            raise
        raise KinematicsError("coordinate_feedback_invalid", str(error)) from None
    joint_within = all(abs(error) <= joint_tol for error in errors.values())
    model_within = position_error <= position_tol and abs(pitch_error) <= pitch_tol
    return {"estimated": True, "frame": "base_link", "observed_pose": observed,
            "joint_positions": dict(feedback), "joint_errors": errors,
            "joint_within_tolerance": joint_within, "position_error_cm": position_error,
            "position_error_vector_cm": vector, "pitch_error_deg": pitch_error,
            "model_within_tolerance": model_within,
            "within_tolerance": joint_within and model_within,
            "position_tolerance_cm": position_tol, "pitch_tolerance_deg": pitch_tol,
            "joint_tolerance": joint_tol, "observed_at": observed_at, "score": score}


def _limits(value: dict) -> dict[int, tuple[int, int]]:
    if type(value) is not dict:
        raise ValueError("limits 必须为关节范围对象")
    result = {}
    for key, bounds in value.items():
        if type(key) is str and key in {str(i) for i in range(1, 7)}:
            servo_id = int(key)
        elif type(key) is int and 1 <= key <= 6:
            servo_id = key
        else:
            raise ValueError("limits 的关节编号无效")
        if (servo_id in result or type(bounds) not in (list, tuple) or len(bounds) != 2
                or any(type(bound) is not int for bound in bounds)
                or not 0 <= bounds[0] <= bounds[1] <= (700 if servo_id == 1 else 1000)):
            raise ValueError("limits 需要唯一编号和有效整数 [min,max]")
        result[servo_id] = tuple(bounds)
    missing = set(_JOINTS) - set(result)
    if missing:
        raise CorrectionError("correction_limit", "缺少定位关节限位", {"missing_ids": sorted(missing)})
    return result


def plan_correction(model: KinematicsModel, desired_targets: dict[int, int],
                    last_command_targets: dict[int, int], feedback_positions: dict[int, int],
                    limits: dict[int, tuple[int, int]], start_positions: dict[int, int], *,
                    max_step: int = 5, max_offset: int = 10, max_joint_step: int = 30,
                    deadband: int = 1) -> dict[int, int]:
    """Propose a whole, bounded command from measured signed joint errors.

    Each error is desired minus feedback, added to the previous command after
    step limiting. All axes are checked before returning; no clipping to joint
    limits, partial command, fixed offset or I/O is performed.
    """
    try:
        step = _integer(max_step, "max_step", minimum=1)
        offset = _integer(max_offset, "max_offset")
        joint_step = _integer(max_joint_step, "max_joint_step", minimum=1)
        band = _integer(deadband, "deadband")
        desired = _positions(desired_targets, "desired_targets", only_controlled=True)
        previous = _positions(last_command_targets, "last_command_targets", only_controlled=True)
        feedback = _positions(feedback_positions, "feedback_positions")
        start = _positions(start_positions, "start_positions")
        bounds = _limits(limits)
        # Validate feedback even though FK is not used to choose the correction.
        model.forward(feedback)
    except CorrectionError:
        raise
    except (ValueError, OverflowError, TypeError) as error:
        raise CorrectionError("correction_invalid", str(error)) from None
    candidate = dict(previous)
    for servo_id in _JOINTS:
        error = desired[servo_id] - feedback[servo_id]
        if abs(error) > band:
            candidate[servo_id] += max(-step, min(step, error))
    if candidate == previous:
        raise CorrectionError("correction_no_change", "反馈误差处于死区内，没有新的修正目标")
    violations = [{"servo_id": i, "position": candidate[i], "min": bounds[i][0], "max": bounds[i][1]}
                  for i in _JOINTS if not bounds[i][0] <= candidate[i] <= bounds[i][1]]
    if violations:
        raise CorrectionError("correction_limit", "修正目标超出原始关节范围，整条命令已拒绝",
                              {"candidate_targets": candidate, "violations": violations})
    try:
        model.check_nominal(candidate)
    except KinematicsError as error:
        raise CorrectionError("correction_nominal_limit", "修正目标超出模型名义范围，整条命令已拒绝",
                              {"candidate_targets": candidate, "model_error": error.details}) from None
    excessive_offset = {i: candidate[i] - desired[i] for i in _JOINTS if abs(candidate[i] - desired[i]) > offset}
    if excessive_offset:
        raise CorrectionError("correction_offset_limit", "修正目标相对原目标的累计偏移超过限制",
                              {"candidate_targets": candidate, "offsets": excessive_offset, "max_offset": offset})
    excessive_step = {i: candidate[i] - start[i] for i in _JOINTS if abs(candidate[i] - start[i]) > joint_step}
    if excessive_step:
        raise CorrectionError("correction_joint_step", "修正目标相对本任务起点的变化超过限制",
                              {"candidate_targets": candidate, "joint_deltas": excessive_step,
                               "max_joint_step": joint_step})
    return candidate
