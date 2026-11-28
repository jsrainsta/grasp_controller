"""Train and evaluate the three-class medicine-box detector on the local export."""
import csv
import json
import os
import shutil
import sys
from pathlib import Path

RUN = Path(__file__).resolve().parent
YOLO_DIR = RUN.parent
SOURCE = YOLO_DIR / "dataset"
DATA = RUN / "data"
sys.path.insert(0, str(YOLO_DIR / "board_seg/work/libs"))
os.environ.update(YOLO_OFFLINE="true", YOLO_AUTOINSTALL="false", OMP_NUM_THREADS="8",
                  MKL_NUM_THREADS="8", YOLO_CONFIG_DIR=str(RUN / "config"))

from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer


class LocalTrainer(DetectionTrainer):
    def read_results_csv(self):
        with self.csv.open(encoding="utf8") as handle:
            rows = list(csv.DictReader(handle))
        return {key: [float(row[key]) for row in rows] for key in rows[0]} if rows else {}


def prepare():
    header, *images = map(json.loads, (SOURCE / "medicine-boxes.ndjson").read_text("utf8").splitlines())
    names = {int(index): name for index, name in header["class_names"].items()}
    seen = set()
    for record in images:
        split, filename = record["split"], record["file"]
        if split not in {"train", "val", "test"} or Path(filename).name != filename or filename in seen:
            raise ValueError(f"Invalid dataset entry: {split}/{filename}")
        seen.add(filename)
        image_path = DATA / "images" / split / filename
        label_path = DATA / "labels" / split / Path(filename).with_suffix(".txt")
        image_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SOURCE / "compressed_640480_detect" / filename, image_path)
        boxes = record.get("annotations", {}).get("boxes", [])
        for cls, x, y, width, height in boxes:
            if (cls not in names or min(width, height) <= 0
                    or min(x-width/2, y-height/2) < -1e-4
                    or max(x+width/2, y+height/2) > 1.0001):
                raise ValueError(f"Invalid box in {filename}")
        label_path.write_text("".join(" ".join(map(str, box))+"\n" for box in boxes), encoding="utf8")
    yaml = RUN / "dataset.yaml"
    lines = [f"path: {json.dumps(DATA.as_posix(), ensure_ascii=False)}"]
    lines.extend(f"{split}: images/{split}" for split in ("train", "val", "test"))
    lines.append("names:")
    lines.extend(f"  {index}: {json.dumps(name, ensure_ascii=False)}" for index, name in names.items())
    yaml.write_text("\n".join(lines)+"\n", encoding="utf8")
    return yaml


def main():
    yaml = prepare()
    checkpoint = RUN / "exp_v5_detect/weights/best.pt"
    if "--evaluate" not in sys.argv:
        YOLO(str(RUN / "yolo26n.pt")).train(
            trainer=LocalTrainer, data=str(yaml), project=str(RUN), name="exp_v5_detect",
            epochs=100, patience=0, imgsz=416, batch=8, workers=0, device="cpu",
            freeze=10, amp=False, seed=26, plots=False, verbose=False,
            mosaic=.25, close_mosaic=10, fliplr=0., degrees=8, cache="ram")
    model = YOLO(str(checkpoint))
    metrics = model.val(data=str(yaml), split="test", imgsz=416, batch=8, device="cpu",
                        workers=0, plots=True, project=str(RUN), name="test")
    report = {"overall": metrics.results_dict, "classes": {
        model.names[index]: dict(zip(("precision", "recall", "mAP50", "mAP50-95"),
                                    map(float, metrics.class_result(i))))
        for i, index in enumerate(metrics.ap_class_index)}}
    (RUN / "test_metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf8")
    shutil.copyfile(checkpoint, YOLO_DIR / "exp_v5_detect.pt")


if __name__ == "__main__":
    main()
