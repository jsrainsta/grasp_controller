"""Download a reproducible sample of public medicine cartons and their original labels."""
import io
import json
import random
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests
from PIL import Image
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry


ROOT = Path(__file__).resolve().parents[1] / "work" / "public"
REPO = "Malek-Messaoudi/Medicine_boxes_annotated"
HOST = "https://huggingface.co"


def get(url):
    with requests.Session() as client:
        client.mount("https://", HTTPAdapter(max_retries=Retry(total=3, backoff_factor=0.5,
                     status_forcelist=(429, 500, 502, 503, 504))))
        response = client.get(url, timeout=40)
        response.raise_for_status()
        return response


def main():
    ROOT.mkdir(exist_ok=True, parents=True)
    revision = "c32be90ad070c64f72c5ac41c95409b3edda4f09"
    listing = f"{HOST}/api/datasets/{REPO}/tree/{revision}/train"
    images = get(listing + "/images?limit=1000").json()
    labels = {Path(row["path"]).stem: row["path"] for row in get(listing + "/labels?limit=1000").json()}
    paired = sorted((row["path"], labels[Path(row["path"]).stem]) for row in images
                    if Path(row["path"]).stem in labels)
    sample = random.Random(261119).sample(paired, min(120, len(paired)))
    base = f"{HOST}/datasets/{REPO}/resolve/{revision}/"

    def fetch(pair):
        photo, label = pair
        stem = Path(photo).stem
        annotation = get(base + label).text.strip()
        rows = [line.split() for line in annotation.splitlines()]
        if not rows or any(len(row) < 7 or len(row) % 2 != 1 for row in rows):
            return None
        im = Image.open(io.BytesIO(get(base + photo).content)).convert("RGB")
        im.thumbnail((640, 640))
        im.save(ROOT / (stem + ".jpg"), quality=92)
        (ROOT / (stem + ".txt")).write_text(annotation, encoding="utf8")
        return {"id": stem, "image": photo, "annotation": label}

    with ThreadPoolExecutor(6) as pool:
        items = [row for row in pool.map(fetch, sample) if row]
    (ROOT / "source.json").write_text(json.dumps(dict(repo=REPO, revision=revision, items=items),
                                                indent=2), encoding="utf8")
    print(f"{len(items)} images with polygon-form labels")


if __name__ == "__main__":
    main()
