"""CPU fine-tuning with only the small Ultralytics wheel layered over the existing torch."""
import argparse
import csv
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"work/libs"))
os.environ["YOLO_CONFIG_DIR"] = str(ROOT/"work/config")
os.environ["YOLO_OFFLINE"] = "true"
os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["OMP_NUM_THREADS"] = "6"
os.environ["MKL_NUM_THREADS"] = "6"

from ultralytics import YOLO
from ultralytics.models.yolo.segment import SegmentationTrainer


class LocalTrainer(SegmentationTrainer):
    # Avoid a 100 MB dataframe library solely for embedding numeric CSV data in checkpoints.
    def read_results_csv(self):
        with self.csv.open(encoding="utf8") as handle:
            rows = list(csv.DictReader(handle))
        return {key: [float(row[key]) for row in rows] for key in rows[0]} if rows else {}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", choices=("local","mixed","paired"))
    parser.add_argument("--epochs", type=int, default=12)
    args = parser.parse_args()
    paired = args.experiment=="paired"
    initial = ROOT/"runs/mixed/weights/best.pt" if paired else ROOT/"work/yolo26n-seg.pt"
    YOLO(str(initial)).train(
        trainer=LocalTrainer, data=str(ROOT/f"work/dataset/{args.experiment}.yaml"),
        project=str(ROOT/"runs"), name=args.experiment, device="cpu", epochs=args.epochs,
        imgsz=384, batch=6, workers=0, seed=26, deterministic=True, freeze=10,
        optimizer="AdamW", lr0=.0004 if paired else .001, warmup_epochs=1, mosaic=.25, close_mosaic=3,
        degrees=12, translate=.10, scale=.25, amp=False, plots=False, cache=False,
        patience=6, verbose=False)
