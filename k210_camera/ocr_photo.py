"""Run local Chinese OCR on one received photo; no serial or arm access."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

if __package__:
    from .medicine_match import load_catalog, match_medicine
    from .target_selection import resolve_target, select_target
else:
    from medicine_match import load_catalog, match_medicine
    from target_selection import resolve_target, select_target


PROJECT_DIR = Path(__file__).resolve().parent


def latest_photo(directory):
    photos = [path for path in directory.glob("*")
              if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}]
    if not photos:
        raise FileNotFoundError("没有找到照片，请先按 K1 接收一张，或在命令中指定图片路径。")
    return max(photos, key=lambda path: path.stat().st_mtime_ns)


def text_lines(result):
    texts = getattr(result, "txts", None)
    scores = getattr(result, "scores", None)
    boxes = getattr(result, "boxes", None)
    if texts is None or len(texts) == 0:
        return []
    if scores is None or boxes is None or not (len(texts) == len(scores) == len(boxes)):
        raise ValueError("OCR 输出的文字、分数和文本框数量不一致。")
    lines = []
    for text, score, box in zip(texts, scores, boxes):
        confidence = float(score)
        points = [[float(value) for value in point] for point in box]
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("OCR 返回了无效置信度。")
        if len(points) != 4 or any(len(point) != 2 for point in points):
            raise ValueError("OCR 返回了无效文本框。")
        if any(not math.isfinite(value) for point in points for value in point):
            raise ValueError("OCR 文本框包含无效坐标。")
        lines.append({"text": str(text), "confidence": confidence, "polygon": points})
    return lines


def recognize_orientations(engine, image, catalog, min_score):
    """Retry unresolved names at right angles; keep all evidence, never guess.

    Polygons are mapped back to the original displayed image. Matching distinct
    drug names across attempts remains ambiguous, regardless of requested target.
    """
    width, height = image.size
    lines, attempts = [], []
    def attempt(angle):
        rotated = image if angle == 0 else image.rotate(angle, expand=True)
        detected = text_lines(engine(rotated))
        match = match_medicine(detected, catalog, min_score)
        attempts.append({'rotation_ccw_deg': angle, 'line_count': len(detected),
                         'match_status': match['status'], 'medicine_id': match['medicine_id']})
        for line in detected:
            mapped = []
            for x, y in line['polygon']:
                if angle == 90:
                    x, y = width-1-y, x
                elif angle == 180:
                    x, y = width-1-x, height-1-y
                elif angle == 270:
                    x, y = y, height-1-x
                mapped.append([x, y])
            lines.append({**line, 'polygon': mapped, 'ocr_rotation_ccw_deg': angle})
        return match
    initial = attempt(0)
    if initial['status'] in ('no_text', 'unknown', 'low_confidence'):
        print('原方向尚未确认药名，补充检查旋转方向（复用同一张照片）。', flush=True)
        for angle in (270, 90, 180):
            attempt(angle)
    return lines, attempts


class OCRRunner:
    """Reuse a catalog and local OCR engine for sequential, explicit photo paths."""

    def __init__(self, catalog_path=None, min_name_score=0.8, target_id=None):
        if not math.isfinite(min_name_score) or not 0 <= min_name_score <= 1:
            raise ValueError("min_name_score 必须在 0 到 1 之间。")
        self.catalog_path = Path(
            catalog_path or PROJECT_DIR / "medicines.json").expanduser().resolve()
        self.catalog = load_catalog(self.catalog_path)
        self.min_name_score = min_name_score
        self.target = resolve_target(self.catalog, target_id)
        self._engine = None
        self._has_run = False

    def _get_engine(self):
        # Keep imports and engine setup lazy for --help and invalid photo paths.
        if self._engine is None:
            from rapidocr import RapidOCR

            self._engine = RapidOCR(params={
                "Global.log_level": "warning",
                "EngineConfig.onnxruntime.intra_op_num_threads": 2,
                "EngineConfig.onnxruntime.inter_op_num_threads": 1,
            })
        return self._engine

    def run(self, image_path, output_dir, known_box_focus=False):
        from PIL import Image, ImageOps

        image_path = Path(image_path).expanduser().resolve()
        output_dir = Path(output_dir).expanduser().resolve()
        if not image_path.is_file():
            raise FileNotFoundError("找不到图片：" + str(image_path))
        with Image.open(image_path) as source:
            image = ImageOps.exif_transpose(source).convert("RGB")
        width, height = image.size
        print("图片：", image_path)
        print("尺寸：{}×{}".format(width, height))
        first_run = not self._has_run
        if first_run:
            print("正在首次加载中文 OCR 模型并识别……", flush=True)
        else:
            print("正在识别（复用已加载的中文 OCR 模型）……", flush=True)
        prepare_started = time.perf_counter()
        engine = self._get_engine()
        started = time.perf_counter()
        model_prepare = started - prepare_started
        lines, orientation_attempts = recognize_orientations(engine, image, self.catalog, self.min_name_score)
        focus_record=None
        if known_box_focus and match_medicine(lines,self.catalog,self.min_name_score)['status'] in ('no_text','unknown','low_confidence'):
            # Current blue/white box only: color proposes a bounded ROI, but
            # identity still requires the complete OCR name. Never guess it.
            import cv2
            import numpy as np
            rgb=np.array(image);hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
            mask=cv2.inRange(hsv,np.array([80,65,55]),np.array([125,255,255]))
            ys,xs=np.where(mask>0)
            if len(xs)>300:
                x0,x1=int(xs.min()),int(xs.max());y0,y1=int(ys.min()),int(ys.max())
                padx=max(12,round((x1-x0)*.5));pady=max(12,round((y1-y0)*.15))
                x0,x1=max(0,x0-padx),min(width,x1+padx+1)
                y0,y1=max(0,y0-pady),min(height,y1+pady+1)
                if (x1-x0)*(y1-y0)<=width*height*.45:
                    crop=image.crop((x0,y0,x1,y1));scale=3
                    crop=crop.resize((crop.width*scale,crop.height*scale),Image.Resampling.LANCZOS)
                    extra,attempts=recognize_orientations(engine,crop,self.catalog,self.min_name_score)
                    for line in extra:
                        lines.append({**line,'polygon':[[x/scale+x0,y/scale+y0] for x,y in line['polygon']],
                                      'ocr_source':'known_box_color_roi_3x'})
                    focus_record={'source_bbox_px':[x0,y0,x1,y1],'scale':scale,
                                  'adds_physical_detail':False,'complete_name_required':True}
                    orientation_attempts.extend({**v,'source':'known_box_color_roi_3x'} for v in attempts)
        elapsed = time.perf_counter() - started
        self._has_run = True
        for index, line in enumerate(lines, start=1):
            print("{}. {}  （置信度 {:.2f}）".format(index, line["text"], line["confidence"]))
        if not lines:
            print("这张照片没有识别出文字，请检查清晰度、反光和文字大小。")
        timing_note = "首张识别" if first_run else "复用已加载模型"
        print("模型准备耗时：{:.2f} 秒。".format(model_prepare))
        print("OCR 推理耗时：{:.2f} 秒（{}）。".format(elapsed, timing_note))
        medicine_match = match_medicine(lines, self.catalog, self.min_name_score)
        match_status = medicine_match["status"]
        if match_status == "matched":
            print("名称匹配：{}（演示编号 {}）".format(
                medicine_match["name"], medicine_match["medicine_id"]))
        elif match_status == "ambiguous":
            print("名称匹配：出现多个名单内药品名称，暂未选择目标。")
        elif match_status == "low_confidence":
            print("名称匹配：读到了名单内名称，但模型分数偏低，暂未确认。")
        elif match_status == "no_text":
            print("名称匹配：没有文字，暂未确认。")
        else:
            print("名称匹配：没有读到名单中的完整名称，暂未确认。")

        target_selection = select_target(medicine_match, self.target)
        if target_selection is not None:
            if target_selection["status"] == "target_found":
                print("目标筛选：目标药名匹配「{}」。".format(self.target["name"]))
            elif target_selection["status"] == "other_medicine":
                print("目标筛选：识别到其他药品「{}」，当前目标为「{}」。".format(
                    target_selection["recognized_name"], self.target["name"]))
            else:
                print("目标筛选：暂未确认（当前目标：{}）。".format(self.target["name"]))

        report = {
            "type": "ocr_result",
            "image_path": str(image_path),
            "image_width": width,
            "image_height": height,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "engine": "RapidOCR / ONNX Runtime CPU",
            "versions": {name: importlib.metadata.version(name)
                         for name in ("rapidocr", "onnxruntime")},
            "status": "text_found" if lines else "no_text",
            "elapsed_s": elapsed,
            "model_prepare_s": model_prepare,
            "lines": lines,
            "orientation_attempts": orientation_attempts,
            "known_box_focus": focus_record,
            "polygon_coordinate_frame": "original_exif_transposed_image_pixels",
            "orientation_fallback_policy": "retry_unconfirmed_names_at_right_angles_keep_all_evidence",
            "catalog_path": str(self.catalog_path),
            "min_name_score": self.min_name_score,
            "medicine_match": medicine_match,
            "target_selection": target_selection,
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / (image_path.stem + "_ocr.json")
        text_path = output_dir / (image_path.stem + "_ocr.txt")
        json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
                             encoding="utf-8")
        text_path.write_text("\n".join(line["text"] for line in lines) + ("\n" if lines else ""),
                             encoding="utf-8")
        print("文字结果：", text_path)
        print("结构化结果：", json_path)
        return report


def run_ocr(image_path, output_dir, catalog_path=None, min_name_score=0.8,
            target_id=None):
    return OCRRunner(catalog_path, min_name_score, target_id).run(image_path, output_dir)


def main():
    parser = argparse.ArgumentParser(description="读取一张 K210 照片，输出中文 OCR 文字")
    parser.add_argument("image", nargs="?", type=Path,
                        help="图片路径；省略时读取 images 目录里最新照片")
    parser.add_argument("--output", type=Path, default=PROJECT_DIR / "ocr_results")
    parser.add_argument("--catalog", type=Path, default=PROJECT_DIR / "medicines.json",
                        help="演示药品名称名单")
    parser.add_argument("--min-name-score", type=float, default=0.8,
                        help="完整名称匹配所需的最低 OCR 模型分数，默认 0.8")
    parser.add_argument("--target", help="筛选目标药品的演示编号，例如 med_002；省略时只做名称匹配")
    args = parser.parse_args()
    image_path = (args.image or latest_photo(PROJECT_DIR / "images")).expanduser().resolve()
    if not image_path.is_file():
        raise FileNotFoundError("找不到图片：" + str(image_path))
    if not math.isfinite(args.min_name_score) or not 0 <= args.min_name_score <= 1:
        raise ValueError("--min-name-score 必须在 0 到 1 之间。")
    run_ocr(image_path, args.output.expanduser().resolve(), args.catalog,
            args.min_name_score, target_id=args.target)


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as error:
        print("缺少 OCR 依赖：", error.name)
        print("请按 OCR实验说明.md 安装 requirements-ocr.txt 后重试。")
        raise SystemExit(1)
    except (OSError, ValueError) as error:
        print("OCR 未完成：", error)
        raise SystemExit(1)
