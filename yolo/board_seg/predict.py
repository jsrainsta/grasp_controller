"""Offline medicine-box segmentation on a saved image or a folder of images."""
import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/"work/libs"))
os.environ["YOLO_CONFIG_DIR"] = str(ROOT/"work/config")
os.environ["YOLO_OFFLINE"] = "true"
os.environ["YOLO_AUTOINSTALL"] = "false"

from ultralytics import YOLO


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path)
    parser.add_argument("--variant",choices=("mixed","local"),default="mixed")
    args=parser.parse_args()
    if not args.source.exists():
        parser.error(f"Image or folder does not exist: {args.source}")
    config=json.loads((ROOT/"model_settings.json").read_text(encoding="utf8"))[args.variant]
    model=YOLO(str(ROOT/config["weights"]))
    results=model.predict(str(args.source),stream=True,imgsz=config["imgsz"],conf=config["conf"],
                          device="cpu",retina_masks=True,save=True,project=str(ROOT/"previews"),
                          name="medicine_boxes",verbose=False)
    for r in results:
        print(f"{Path(r.path).name}: {len(r.boxes)} medicine box(es); {r.save_dir}")


if __name__=="__main__":
    main()
