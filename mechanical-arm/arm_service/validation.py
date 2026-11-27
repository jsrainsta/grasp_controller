"""严格校验 HTTP JSON；拒绝布尔值充当整数和未知字段。"""

from __future__ import annotations

import re
from decimal import Decimal, InvalidOperation

from .errors import ArmError


def invalid(message: str) -> None:
    raise ArmError("invalid_request", message, 422)


def fields(data, required=(), optional=()) -> dict:
    if type(data) is not dict:
        invalid("请求体必须为 JSON 对象")
    if set(data) - set(required) - set(optional):
        invalid("包含未知字段：" + ", ".join(sorted(set(data) - set(required) - set(optional))))
    if set(required) - set(data):
        invalid("缺少字段：" + ", ".join(sorted(set(required) - set(data))))
    return data


def integer(value, low: int, high: int, name: str) -> int:
    if type(value) is not int or not low <= value <= high:
        invalid(f"{name} 必须为 {low}～{high} 的整数")
    return value


def request_id(value) -> str:
    if type(value) is not str or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", value):
        invalid("request_id 必须为 1～128 字符，只含字母、数字、下划线、点或短横线")
    return value


def targets(value) -> dict[int, int]:
    if type(value) is not list or not 1 <= len(value) <= 6:
        invalid("targets 必须包含 1～6 个舵机目标")
    result = {}
    for item in value:
        fields(item, ("id", "position"))
        servo_id = integer(item["id"], 1, 6, "id")
        position = integer(item["position"], 0, 700 if servo_id == 1 else 1000, "position")
        if servo_id in result:
            invalid("targets 不能包含重复舵机 ID")
        result[servo_id] = position
    return result


def gripper_parameters(close_target, expected_contact_range) -> dict:
    """HTTP 与服务直调共用的夹爪参数校验，不操作设备。"""
    target = integer(close_target, 0, 700, "close_target")
    fields(expected_contact_range, ("min", "max"))
    lower = integer(expected_contact_range["min"], 0, 700, "expected_contact_range.min")
    upper = integer(expected_contact_range["max"], 0, 700, "expected_contact_range.max")
    if lower > upper:
        invalid("expected_contact_range.min 不得大于 max")
    return {"close_target": target, "expected_contact_range": {"min": lower, "max": upper}}


def delta(value, name: str) -> int:
    if type(value) not in (int, float):
        invalid(f"{name} 必须为以厘米为单位的数值")
    try:
        ticks = Decimal(str(value)) * 10
        if not ticks.is_finite() or ticks != ticks.to_integral_value() or not -128 <= ticks <= 127:
            invalid(f"{name} 必须为 -12.8～12.7 cm，且为 0.1 cm 的整数倍")
        return int(ticks)
    except InvalidOperation:
        invalid(f"{name} 为无效数值")


def parse_command(name: str, data: dict) -> dict:
    required = {"move": ("request_id", "targets"), "reset": ("request_id",),
                "stop": ("request_id",), "reconnect": ("request_id",),
                "task_release": ("request_id", "confirm_stationary"),
                "gripper_close": ("request_id", "close_target", "expected_contact_range"),
                "coordinate": ("request_id", "dx_cm", "dy_cm", "dz_cm"),
                "coordinate_preview": ("request_id", "dx_cm", "dy_cm", "dz_cm"),
                "group_run": ("request_id", "repeat"),
                "group_upload": ("request_id", "frames"),
                "group_erase": ("request_id",),
                "offset_set": ("request_id", "offsets"), "offset_save": ("request_id",)}
    optional = ("duration_ms", "position_tolerance") if name == "move" else ()
    fields(data, required[name], optional)
    request_id(data["request_id"])
    result = dict(data)
    if name == "move":
        result["targets"] = targets(data["targets"])
        result["duration_ms"] = integer(data.get("duration_ms", 20), 20, 20, "duration_ms（出厂总线直接控制固定为 20）")
        result["position_tolerance"] = (integer(data["position_tolerance"], 0, 100, "position_tolerance")
                                        if "position_tolerance" in data else None)
    elif name == "task_release":
        if data["confirm_stationary"] is not True:
            invalid("confirm_stationary 必须为 true，且须先在现场确认已静止")
    elif name == "gripper_close":
        result.update(gripper_parameters(data["close_target"], data["expected_contact_range"]))
    elif name in {"coordinate", "coordinate_preview"}:
        result["delta_ticks"] = tuple(delta(data[key], key) for key in ("dx_cm", "dy_cm", "dz_cm"))
        if not any(result["delta_ticks"]):
            invalid("坐标增量不能全部为零")
    elif name == "group_run":
        result["repeat"] = integer(data["repeat"], 1, 255, "repeat")
    elif name == "group_upload":
        if type(data["frames"]) is not list or not 1 <= len(data["frames"]) <= 255:
            invalid("frames 必须包含 1～255 个动作帧")
        result["frames"] = []
        for frame in data["frames"]:
            fields(frame, ("targets", "duration_ms"))
            result["frames"].append({"targets": targets(frame["targets"]),
                                     "duration_ms": integer(frame["duration_ms"], 1, 65535, "duration_ms")})
    elif name == "offset_set":
        if type(data["offsets"]) is not list or not 1 <= len(data["offsets"]) <= 6:
            invalid("offsets 必须包含 1～6 项")
        offsets = {}
        for item in data["offsets"]:
            fields(item, ("id", "offset"))
            servo_id = integer(item["id"], 1, 6, "id")
            if servo_id in offsets:
                invalid("offsets 不能包含重复舵机 ID")
            offsets[servo_id] = integer(item["offset"], -100, 100, "offset")
        result["offsets"] = offsets
    return result
