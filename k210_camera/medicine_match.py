"""Conservative, local matching of complete OCR lines to a medicine catalog.

OCR confidence is used only as a threshold; it is not proof of medicine identity.
No fuzzy matching, missing-character completion, or substring matching is used.
"""

import json
import math
import unicodedata
from pathlib import Path


def normalise_text(text):
    """Normalise width, whitespace and letter case while preserving punctuation."""
    if not isinstance(text, str):
        raise ValueError("文字必须是字符串。")
    return "".join(unicodedata.normalize("NFKC", text).split()).casefold()


def _validate_catalog(catalog):
    if not isinstance(catalog, list) or not catalog:
        raise ValueError("medicines 必须是非空列表。")
    validated = []
    seen_ids = set()
    seen_names = set()
    for index, item in enumerate(catalog):
        if not isinstance(item, dict):
            raise ValueError("第 {} 个药品条目必须是对象。".format(index + 1))
        medicine_id = item.get("id")
        name = item.get("name")
        if not isinstance(medicine_id, str) or not medicine_id.strip():
            raise ValueError("第 {} 个药品的 id 必须是非空字符串。".format(index + 1))
        if not isinstance(name, str) or not normalise_text(name):
            raise ValueError("第 {} 个药品的 name 必须是非空字符串。".format(index + 1))
        normalised_name = normalise_text(name)
        if medicine_id in seen_ids:
            raise ValueError("药品 id 重复：{}".format(medicine_id))
        if normalised_name in seen_names:
            raise ValueError("药品名称规范化后重复：{}".format(name))
        seen_ids.add(medicine_id)
        seen_names.add(normalised_name)
        validated.append({"id": medicine_id, "name": name})
    return validated


def load_catalog(path):
    """Load version 1 JSON, allowing a UTF-8 BOM added by Windows editors."""
    with Path(path).open("r", encoding="utf-8-sig") as source:
        data = json.load(source)
    if not isinstance(data, dict):
        raise ValueError("药品目录必须是 JSON 对象。")
    if type(data.get("version")) is not int or data["version"] != 1:
        raise ValueError("药品目录 version 必须是整数 1。")
    return _validate_catalog(data.get("medicines"))


def _validate_score(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} 必须是 0 到 1 之间的数值。".format(field))
    try:
        score = float(value)
    except OverflowError as error:
        raise ValueError("{} 必须是 0 到 1 之间的有限数值。".format(field)) from error
    if not math.isfinite(score) or not 0.0 <= score <= 1.0:
        raise ValueError("{} 必须是 0 到 1 之间的有限数值。".format(field))
    return score


def match_medicine(lines, catalog, min_score=0.8):
    """Match complete OCR lines; return an auditable JSON-serialisable result.

    Line indexes are zero-based. Evidence from repeated lines is grouped by
    medicine ID, so repeated observations of one name are not ambiguous.
    """
    threshold = _validate_score(min_score, "min_score")
    medicines = _validate_catalog(catalog)
    if not isinstance(lines, list):
        raise ValueError("OCR lines 必须是列表。")
    by_name = {normalise_text(item["name"]): item for item in medicines}
    candidates_by_id = {}
    has_text = False
    for index, line in enumerate(lines):
        if not isinstance(line, dict) or not isinstance(line.get("text"), str):
            raise ValueError("第 {} 行 OCR 结果必须包含字符串 text。".format(index + 1))
        score = _validate_score(line.get("confidence"), "第 {} 行 confidence".format(index + 1))
        normalised = normalise_text(line["text"])
        if not normalised:
            continue
        has_text = True
        medicine = by_name.get(normalised)
        if medicine is None:
            continue
        candidate = candidates_by_id.setdefault(
            medicine["id"],
            {
                "medicine_id": medicine["id"],
                "name": medicine["name"],
                "ocr_confidence": score,
                "source_line_indices": [],
                "evidence": [],
                "meets_threshold": False,
            },
        )
        candidate["ocr_confidence"] = max(candidate["ocr_confidence"], score)
        candidate["source_line_indices"].append(index)
        candidate["evidence"].append(
            {
                "line_index": index,
                "text": line["text"],
                "ocr_confidence": score,
                "meets_threshold": score >= threshold,
            }
        )
        candidate["meets_threshold"] = candidate["meets_threshold"] or score >= threshold

    candidates = list(candidates_by_id.values())
    qualifying = [item for item in candidates if item["meets_threshold"]]
    result = {
        "status": "unknown",
        "medicine_id": None,
        "name": None,
        "ocr_confidence": None,
        "source_line_index": None,
        "source_line_indices": [],
        "min_score": threshold,
        "candidates": candidates,
        "reason": "识别出了文字，但没有与目录完整药名一致且达到阈值的结果。",
    }
    if len(qualifying) >= 2:
        result["status"] = "ambiguous"
        result["reason"] = "多个不同药品名称完整匹配且达到阈值，暂不选择药品。"
    elif len(qualifying) == 1:
        selected = qualifying[0]
        best_evidence = max(selected["evidence"], key=lambda item: item["ocr_confidence"])
        result.update(
            status="matched",
            medicine_id=selected["medicine_id"],
            name=selected["name"],
            ocr_confidence=selected["ocr_confidence"],
            source_line_index=best_evidence["line_index"],
            source_line_indices=[
                item["line_index"] for item in selected["evidence"] if item["meets_threshold"]
            ],
            reason="完整药名与目录一致，OCR 置信度达到阈值。",
        )
    elif candidates:
        result["status"] = "low_confidence"
        result["reason"] = "完整药名与目录一致，但 OCR 置信度未达到阈值，暂不确认。"
    elif not has_text:
        result["status"] = "no_text"
        result["reason"] = "没有识别出文字。"
    return result
