"""Package only selected samples with portable split lists and check contents."""
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/"work"
DATA=WORK/"dataset"


def main():
    splits={s:[Path(x) for x in (DATA/(s+".txt")).read_text(encoding="utf8").splitlines()]
            for s in ("train","mixed","paired","val","test")}
    images=sorted({p for paths in splits.values() for p in paths})
    digests={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in images}
    train={digests[p] for s in ("train","mixed","paired") for p in splits[s]}
    val={digests[p] for p in splits["val"]}
    test={digests[p] for p in splits["test"]}
    if train & val or train & test or val & test:
        raise ValueError("Duplicate image content across dataset partitions")
    manifest={s:len(v) for s,v in splits.items()}
    manifest["images"]={p.relative_to(DATA).as_posix():digests[p] for p in images}
    output=ROOT/"medicine_box_seg_dataset.zip"
    with ZipFile(output,"w",ZIP_DEFLATED,compresslevel=3) as z:
        for p in images:
            rel=p.relative_to(DATA).as_posix()
            label=DATA/"labels"/p.parent.name/(p.stem+".txt")
            z.write(p,rel)
            z.write(label,label.relative_to(DATA).as_posix())
        for split,paths in splits.items():
            z.writestr(split+".txt","".join("./"+p.relative_to(DATA).as_posix()+"\n" for p in paths))
        for name,trainlist in (("local","train"),("mixed","mixed"),("paired","paired")):
            z.writestr(name+".yaml",f"train: {trainlist}.txt\nval: val.txt\ntest: test.txt\nnames: [medicine_box]\n")
        for p in (WORK/"challenge").rglob("*"):
            if p.is_file():
                z.write(p,p.relative_to(WORK).as_posix())
        z.write(WORK/"public/source.json","public_source.json")
        z.writestr("SOURCE.txt","External photos and starting annotations: Malek Messaoudi, Medicine Boxes Annotated Dataset. MIT per dataset card. https://huggingface.co/datasets/Malek-Messaoudi/Medicine_boxes_annotated\nContours refined for visible packaging. render_ and empty_ files are derived training images. IMG and photo images belong to the project.\n")
        z.writestr("manifest.json",json.dumps(manifest,indent=2))
    with ZipFile(output) as z:
        assert z.testzip() is None
        for split,paths in splits.items():
            names=z.read(split+".txt").decode().splitlines()
            assert len(names)==len(paths)
            for path in names:
                assert path[2:] in z.namelist()
    print("Verified splits", {s:len(v) for s,v in splits.items()},"zip bytes",output.stat().st_size)


if __name__=="__main__":
    main()
