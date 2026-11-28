"""Score held-out boxes at one operating point and save review images."""
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from train import DATA, RUN, YOLO_DIR, YOLO

CONFIDENCE = .25
IOU = .50


def overlap(a, b):
    width, height = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]))
    intersection = width * height
    return intersection / (np.prod(a[2:]-a[:2]) + np.prod(b[2:]-b[:2]) - intersection)


def ground_truth(path, shape):
    h, w = shape[:2]
    label = DATA / "labels/test" / path.with_suffix(".txt").name
    boxes = []
    for line in label.read_text("utf8").splitlines():
        cls, x, y, width, height = map(float, line.split())
        box = np.array([x-width/2, y-height/2, x+width/2, y+height/2]) * [w,h,w,h]
        boxes.append((int(cls), box))
    return boxes


def main():
    model = YOLO(str(YOLO_DIR / "exp_v5_detect.pt"))
    paths = sorted((DATA / "images/test").iterdir())
    reviewed = RUN / "test_predictions"
    reviewed.mkdir(exist_ok=True)
    counts = {name: Counter(tp=0, fp=0, fn=0) for name in model.names.values()}
    outcomes = []
    font = ImageFont.truetype("arial.ttf", 17)
    for result in model.predict([str(p) for p in paths], stream=True, imgsz=416, conf=CONFIDENCE,
                                device="cpu", verbose=False):
        path = Path(result.path)
        expected = ground_truth(path, result.orig_img.shape)
        unmatched = set(range(len(expected)))
        predicted = []
        canvas = Image.fromarray(cv2.cvtColor(result.orig_img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(canvas)
        for cls, box in expected:
            draw.rectangle(box.tolist(), outline="#20e070", width=2)
            draw.text((box[0], max(0, box[1]-18)), f"GT {cls}", font=font,
                      fill="#20e070", stroke_width=2, stroke_fill="black")
        for prediction in sorted(result.boxes, key=lambda item: -float(item.conf.item())):
            cls, confidence = int(prediction.cls.item()), float(prediction.conf.item())
            box = prediction.xyxy[0].cpu().numpy()
            score, index = max(((overlap(box, expected[i][1]), i)
                                for i in unmatched if expected[i][0] == cls), default=(0., None))
            matched = index is not None and float(score) >= IOU
            if matched:
                unmatched.remove(index)
            counts[model.names[cls]]["tp" if matched else "fp"] += 1
            predicted.append({"class": cls, "confidence": confidence, "iou": float(score), "matched": matched})
            ink = "#00cbff" if matched else "#ff6347"
            draw.rectangle(box.tolist(), outline=ink, width=2)
            draw.text((box[0], box[1]+2), f"Pred {cls} {confidence:.2f}", fill=ink,
                      font=font, stroke_width=2, stroke_fill="black")
        for index in unmatched:
            counts[model.names[expected[index][0]]]["fn"] += 1
        outcomes.append({"image":path.name, "ground_truth_count":len(expected), "predictions":predicted,
                         "missed":len(unmatched), "exact":not unmatched and all(p["matched"] for p in predicted)})
        canvas.save(reviewed / f"{path.stem}.jpg", quality=90)
    report = {"confidence":CONFIDENCE,"iou":IOU,"classes":counts,
              "exact_images":sum(row["exact"] for row in outcomes),"images":outcomes}
    (RUN / "test_detections.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
    print(json.dumps({"images":len(outcomes),"exact_images":report["exact_images"],"classes":counts},ensure_ascii=False))


if __name__ == "__main__":
    main()
