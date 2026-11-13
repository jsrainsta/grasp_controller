"""Select a requested medicine from an already completed name match.

This module does not change OCR text, select ambiguous candidates, or issue
movement commands. Only a confirmed, unique name match can satisfy a target.
"""


def _required_text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} 必须是非空字符串。".format(field))
    return value


def resolve_target(catalog, target_id):
    """Return the catalog entry for target_id; None leaves filtering disabled."""
    if target_id is None:
        return None
    _required_text(target_id, "target_id")
    if not isinstance(catalog, list) or not catalog:
        raise ValueError("药品目录必须是非空列表。")
    found = None
    seen_ids = set()
    for item in catalog:
        if not isinstance(item, dict):
            raise ValueError("药品目录条目必须是对象。")
        medicine_id = _required_text(item.get("id"), "目录药品 id")
        name = _required_text(item.get("name"), "目录药品 name")
        if medicine_id in seen_ids:
            raise ValueError("药品目录 id 重复：{}".format(medicine_id))
        seen_ids.add(medicine_id)
        if medicine_id == target_id:
            found = {"id": medicine_id, "name": name}
    if found is None:
        raise ValueError("目标药品编号不在目录中：{}".format(target_id))
    return found


def select_target(medicine_match, target):
    """Report target_found, other_medicine or unconfirmed without guessing."""
    if target is None:
        return None
    if not isinstance(target, dict):
        raise ValueError("target 必须是药品目录条目。")
    target_id = _required_text(target.get("id"), "target.id")
    target_name = _required_text(target.get("name"), "target.name")
    if not isinstance(medicine_match, dict):
        raise ValueError("medicine_match 必须是对象。")
    name_status = medicine_match.get("status")
    allowed_statuses = {"matched", "unknown", "no_text", "low_confidence", "ambiguous"}
    if not isinstance(name_status, str) or name_status not in allowed_statuses:
        raise ValueError("medicine_match.status 无效。")
    result = {
        "status": "unconfirmed",
        "target_id": target_id,
        "target_name": target_name,
        "recognized_id": None,
        "recognized_name": None,
        "name_match_status": name_status,
        "reason": "本张照片尚无唯一且达到阈值的完整药名匹配，暂未确认目标。",
    }
    if name_status == "matched":
        recognized_id = _required_text(medicine_match.get("medicine_id"), "medicine_match.medicine_id")
        recognized_name = _required_text(medicine_match.get("name"), "medicine_match.name")
        result["recognized_id"] = recognized_id
        result["recognized_name"] = recognized_name
        if recognized_id == target_id:
            result["status"] = "target_found"
            result["reason"] = "本张照片唯一完整匹配且达到阈值的药名与指定目标一致。"
        else:
            result["status"] = "other_medicine"
            result["reason"] = "本张照片唯一完整匹配且达到阈值的药名对应其他目录药品。"
    return result
