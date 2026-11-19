"""Stress-test real capture history. Marker overlap is an automatic warning, not mask ground truth."""
import csv
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/"work"
INDEX=WORK/"archive_index.json"


def scan():
    import cv2
    dataset=WORK/"dataset"
    held={Path(s).name for part in ("train","val","test")
          for s in (dataset/(part+".txt")).read_text().splitlines()}
    det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    records=[]
    for p in sorted((ROOT.parents[1]/"k210_camera/vision_grasp_images").glob("*/*.jpg")):
        if p.name in held:
            continue
        print("FILE "+p.name,file=sys.stderr,flush=True)
        raw=p.read_bytes()
        im=cv2.imread(str(p))
        if im is None or im.shape[:2]!=(480,640) or not raw.endswith(b"\xff\xd9"):
            print("invalid image",file=sys.stderr,flush=True)
            continue
        pts,ids,_=det.detectMarkers(im)
        markers=[] if ids is None else [q.reshape(4,2).round().astype(int).tolist()
                                      for q,i in zip(pts,ids.ravel()) if i<17]
        records.append(dict(path=str(p),markers=markers if len(markers)>=2 else []))
    INDEX.write_text(json.dumps(records))


def index():
    proc=subprocess.run([sys.executable,__file__,"scan"],capture_output=True,check=True)
    bad=set()
    name=None
    for line in proc.stderr.decode(errors="replace").splitlines():
        if line.startswith("FILE "):
            name=line[5:]
        elif line.strip() and name:
            bad.add(name)
    rows=[r for r in json.loads(INDEX.read_text()) if Path(r["path"]).name not in bad]
    INDEX.write_text(json.dumps(rows))
    (WORK/"archive_excluded.txt").write_text("\n".join(sorted(bad)))
    print("Valid archive frames",len(rows),"decoder warnings or errors",len(bad))


def evaluate():
    from evaluate import MODELS,OUT,YOLO
    import numpy as np
    import cv2
    selected=json.loads((OUT/"selection.json").read_text())
    records=json.loads(INDEX.read_text())
    rows=[]
    fixed="--common" in sys.argv
    names=["expv2","mixed"] if fixed else list(dict.fromkeys(("expv2",selected["new_choice"],"mixed")))
    for name in names:
        ch=selected["validation"]["mixed" if fixed else name]
        model=YOLO(str(MODELS[name]))
        for rec in records:
            p=Path(rec["path"])
            res=model.predict(str(p),imgsz=ch["size"],conf=ch["conf"],device="cpu",
                              retina_masks=True,verbose=False)[0]
            union=np.zeros(res.orig_shape,bool) if res.masks is None else res.masks.data.cpu().numpy().any(axis=0)
            max_overlap=0
            for q in rec["markers"]:
                poly=np.array(q,np.int32)
                x,y,w,h=cv2.boundingRect(poly)
                region=np.zeros((h,w),np.uint8)
                cv2.fillPoly(region,[poly-[x,y]],1)
                covered=(union[y:y+h,x:x+w]&region.astype(bool)).sum()
                max_overlap=max(max_overlap,float(covered)/max(1,int(region.sum())))
            rows.append(dict(model=name,file=p.name,count=len(res.boxes),markers=len(rec["markers"]),
                             max_marker_overlap=round(max_overlap,4)))
    filename="archive_common.csv" if fixed else "archive_per_image.csv"
    with (OUT/filename).open("w",encoding="utf-8-sig",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for name in names:
        rs=[r for r in rows if r["model"]==name]
        print(name,len(rs),"zero",sum(r["count"]==0 for r in rs),"multiple",sum(r["count"]>1 for r in rs),
              "marker visible",sum(r["markers"]>0 for r in rs),"marker half covered",sum(r["max_marker_overlap"]>=.5 for r in rs))


if __name__=="__main__":
    {"scan":scan,"index":index,"evaluate":evaluate}[sys.argv[1]]()
