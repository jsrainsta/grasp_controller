"""Optional PC segmentation. GRASP_YOLO_PYTHON selects the inference interpreter."""
import atexit
import base64
import json
import os
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

import cv2
import numpy as np
from medicine_match import name_matches

WORKSPACE = Path(__file__).resolve().parents[1]
_executor = None
_worker = None
_worker_key = None


def close_worker():
    global _worker
    if _worker is not None:
        _worker.kill()
        _worker.wait()
        _worker = None


atexit.register(close_worker)


def image_masks(path):
    interpreter = os.environ.get("GRASP_YOLO_PYTHON")
    if not interpreter:
        return None
    path = Path(path).resolve()
    packet = _predict(interpreter, str(path), path.stat().st_mtime_ns, "segment")
    return [cv2.imdecode(np.frombuffer(base64.b64decode(encoded), np.uint8), 0)
            for encoded in packet["masks"]]


def image_detections(path):
    interpreter = os.environ.get("GRASP_YOLO_PYTHON", sys.executable)
    path = Path(path).resolve()
    return _predict(interpreter, str(path), path.stat().st_mtime_ns, "detect")["detections"]


def optional_image_masks(path):
    if path is None:
        return None
    try:
        return image_masks(path)
    except (OSError, RuntimeError, ValueError) as error:
        print(f'YOLO 辅助识别不可用：{error}', file=sys.stderr)
        return None


@lru_cache(maxsize=8)
def _predict(interpreter, path, modified, task):
    global _executor, _worker, _worker_key
    if _worker_key != (interpreter, task):
        close_worker()
    if _worker is None:
        _worker = subprocess.Popen([interpreter, "-u", "-X", "utf8", __file__, "--worker", task],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   text=True, encoding="utf-8")
        _worker_key = (interpreter, task)
    if _executor is None:
        _executor = ThreadPoolExecutor(max_workers=1)
    try:
        _worker.stdin.write(json.dumps(path) + "\n")
        _worker.stdin.flush()
        packet = json.loads(_executor.submit(_worker.stdout.readline).result(timeout=45))
    except Exception as error:
        close_worker()
        raise RuntimeError("YOLO worker unavailable: " + str(error)) from error
    if "error" in packet:
        raise RuntimeError(packet["error"])
    return packet


def polygon_mask(shape, polygon):
    mask = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
    return mask


def coverage(mask, region):
    return np.count_nonzero(mask & region) / max(1, np.count_nonzero(region))


def match_region(masks, region, minimum):
    matches = [mask for mask in masks if coverage(mask, region) >= minimum]
    return matches[0] if len(matches) == 1 else None


def overview_mask(masks, shape, ocr):
    matched = set()
    for line in ocr.get("lines", []):
        if name_matches(line.get("text", ""), "阿昔洛韦乳膏") and line.get("confidence", 0) >= .8:
            region = polygon_mask(shape, line["polygon"])
            matched.update(i for i, mask in enumerate(masks) if coverage(mask, region) >= .85)
    return masks[matched.pop()] if len(matched) == 1 else None


def serve(*, self_check=False, task="segment"):
    # Keep the protocol stream separate from Ultralytics startup messages.
    protocol = sys.stdout
    sys.stdout = sys.stderr
    libs = WORKSPACE / "yolo/board_seg/work/libs"
    if libs.is_dir():
        sys.path.insert(0, str(libs))
    os.environ["YOLO_AUTOINSTALL"] = "false"
    config = TemporaryDirectory(prefix="grasp_yolo_")
    os.environ.setdefault("YOLO_CONFIG_DIR", config.name)
    os.environ["YOLO_OFFLINE"] = "true"
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(min(6, torch.get_num_threads()))
    detecting = task == "detect"
    model = YOLO(str(WORKSPACE / ("yolo/exp_v5_detect.pt" if detecting else "yolo/expv4s.pt")))
    if detecting:
        from grasp_medicine import MEDICINES
        if any(model.names.get(i) != row[1] for i, row in enumerate(MEDICINES)):
            raise ValueError("检测权重类别与药盒配置不一致")
    options = (dict(imgsz=416, conf=.25) if detecting
               else dict(imgsz=640, conf=.65, retina_masks=True))
    if self_check:
        model.predict(np.zeros((480, 640, 3), np.uint8), **options, device="cpu", verbose=False)
        return
    for line in sys.stdin:
        try:
            image = cv2.imdecode(np.fromfile(json.loads(line), dtype=np.uint8), cv2.IMREAD_COLOR)
            result = model.predict(image, **options, device="cpu", verbose=False)[0]
            if detecting:
                packet = {"detections":[{"class_id":int(box.cls.item()), "confidence":box.conf.item(),
                                          "xyxy":box.xyxy[0].tolist()} for box in result.boxes]}
            else:
                masks = [] if result.masks is None else result.masks.data.cpu().numpy()
                packet = {"masks": [base64.b64encode(cv2.imencode(".png", (mask > .5).astype(np.uint8)*255)[1]).decode("ascii")
                                    for mask in masks]}
        except Exception as error:
            packet = {"error": str(error)}
        print(json.dumps(packet), file=protocol, flush=True)


if __name__ == "__main__":
    serve(self_check="--self-check" in sys.argv, task="detect" if "detect" in sys.argv else "segment")
