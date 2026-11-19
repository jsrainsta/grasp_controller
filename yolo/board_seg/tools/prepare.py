"""Prepare local instance masks, validation and held-out robot scenes for both experiments."""
from pathlib import Path
import json
import random
import shutil

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parents[1]
WORK = ROOT / "work"
DATA = WORK / "dataset"
SOURCE = PROJECT / "yolo/dataset"

# Reviewed visible contours in a 320 x 240 view. The date groups below are disjoint.
MANUAL = {
    "20261110_083129": [(103,239),(136,118),(145,91),(155,87),(219,102),(198,239)],
    "20261110_081150": [(114,239),(113,124),(117,95),(124,92),(193,92),(201,239)],
    "20261110_082027": [(47,47),(96,46),(104,63),(96,201),(85,206),(25,198)],
    "20261110_092327": [(119,239),(139,152),(147,140),(201,157),(187,239)],
    "20261110_095915": [(62,34),(109,32),(114,48),(112,179),(105,184),(48,177)],
    "20261111_120650": [(137,24),(183,19),(196,156),(192,169),(141,168)],
    "20261111_130545": [(117,31),(161,27),(176,158),(171,174),(118,177)],
    "20261112_081558": [(136,239),(140,132),(144,128),(197,127),(202,239)],
    "20261112_085314": [(145,16),(188,15),(193,128),(185,146),(139,145)],
    "20261112_092803": [(134,96),(197,92),(212,239),(133,239)],
    "20261112_100730": [(117,82),(192,71),(224,213),(230,239),(135,239)],
    "20261117_094319": [(83,101),(126,68),(267,185),(270,195),(249,217),(236,213),(218,216),(208,228),(86,124)],
    "20261117_094350": [(55,146),(220,75),(238,94),(258,125),(249,140),(83,213),(73,207)],
    "20261117_070146": [(129,107),(196,101),(209,239),(131,239)],
    "20261117_074152": [(192,27),(244,26),(274,186),(266,197),(204,195)],
    "20261117_084113": [(166,58),(220,57),(240,211),(227,210),(221,218),(222,239),(169,239)],
}


def write_item(split, name, image, rows):
    path = DATA / "images" / split / (name + ".jpg")
    label = DATA / "labels" / split / (name + ".txt")
    path.parent.mkdir(parents=True, exist_ok=True)
    label.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(image, Path):
        shutil.copyfile(image, path)
    else:
        if not cv2.imwrite(str(path), image):
            raise OSError(path)
    label.write_text("".join("0 " + " ".join(f"{v:.6f}" for v in row) + "\n" for row in rows), encoding="utf8")
    return path


def board_backgrounds(path, segments):
    # Only take crops disjoint from every annotated box. Keep real markers and texture.
    image = cv2.imread(str(path))
    h, w = image.shape[:2]
    mask = np.zeros((h, w), np.uint8)
    for row in segments:
        pts = (np.array(row).reshape(-1, 2) * [w, h]).astype(np.int32)
        cv2.fillPoly(mask, [pts], 255)
    for name, x in (("left", 90), ("right", 330), ("middle", 210)):
        roi = image[205:400, x:x+185]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if not mask[200:405, x-5:x+190].any() and (gray < 65).mean() > .08:
            yield name, cv2.resize(roi, (320, 320))


def main():
    rows = [json.loads(s) for s in (SOURCE/"medicine-box-allsides.ndjson").read_text(encoding="utf8").splitlines()]
    records = [r for r in rows if r["type"] == "image"]
    raw = SOURCE / "compressed_640x480"
    groups = sorted({r["file"][:17] for r in records if r["file"].startswith("IMG_")})
    # One-minute capture blocks stay together.
    random.Random(261119).shuffle(groups)
    validation = set(groups[:max(1, len(groups)//5)])
    for r in records:
        p = raw / r["file"]
        assert p.is_file() and Image.open(p).size == (r["width"], r["height"])
        segments = []
        for obj in r.get("annotations", {}).get("segments", []):
            assert obj[0] == 0 and len(obj) >= 7 and len(obj) % 2 == 1
            assert all(0 <= v <= 1 for v in obj[1:])
            segments.append(obj[1:])
        if p.name.startswith("IMG_"):
            split = "val" if p.name[:17] in validation else "train"
        elif not segments:
            split = "test"
        else:
            split = "train"
        write_item(split, p.stem, p, segments)
        if split == "train" and p.name.startswith("photo_"):
            for side, crop in board_backgrounds(p, segments):
                write_item("train", f"boardcrop_{p.stem}_{side}", crop, [])
    originals = sorted((PROJECT/"k210_camera").glob("vision_grasp_images/*/*.jpg")) + list(raw.glob("photo_*.jpg"))
    for stamp, pts in MANUAL.items():
        p = next(p for p in originals if p.name.startswith("photo_" + stamp))
        xy = (np.array(pts) / [320, 240]).ravel().tolist()
        write_item("val" if stamp.startswith("20261117") else "test", p.stem, p, [xy])
    for split in ("train","val","test"):
        listing = sorted((DATA/"images"/split).glob("*.jpg"))
        (DATA/(split+".txt")).write_text("".join(str(p)+"\n" for p in listing),encoding="utf8")
        print(split,len(listing))
    for exp in ("local","mixed"):
        cfg = f"path: {DATA.as_posix()}\ntrain: {('train' if exp=='local' else 'mixed')}.txt\nval: val.txt\ntest: test.txt\nnames: [medicine_box]\n"
        (DATA/(exp+".yaml")).write_text(cfg,encoding="utf8")


if __name__ == "__main__":
    main()
