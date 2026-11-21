"""Compare the frozen models with identical production settings, on masks and old camera clips."""
import csv
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

from experiment import BASE, DATA, HERE, OLD, YOLO
from evaluate import aggregate, score
import cv2
import numpy as np
from PIL import Image,ImageDraw


def digest(content):
    return hashlib.sha256(content).hexdigest()


def gt(p):
    im=cv2.imread(str(p))
    h,w=im.shape[:2]
    label=p.parents[2]/"labels"/p.parent.name/(p.stem+".txt")
    masks=[]
    for row in label.read_text().splitlines():
        xy=np.array(list(map(float,row.split()[1:]))).reshape(-1,2)*[w,h]
        m=np.zeros((h,w),np.uint8)
        cv2.fillPoly(m,[xy.round().astype(np.int32)],1)
        masks.append(m.astype(bool))
    return masks


def boundary_distances(truth,predictions):
    distances=[]
    candidates=[]
    for i,a in enumerate(truth):
        for j,b in enumerate(predictions):
            iou=(a & b).sum()/max(1,(a | b).sum())
            if iou>=.5:
                candidates.append((iou,i,j))
    used_true,used_pred=set(),set()
    for _,i,j in sorted(candidates,reverse=True):
        if i in used_true or j in used_pred: continue
        used_true.add(i)
        used_pred.add(j)
        edges=[]
        for mask in (truth[i],predictions[j]):
            m=mask.astype(np.uint8)
            edges.append(m-cv2.erode(m,np.ones((3,3),np.uint8)))
        a,b=edges
        to_a=cv2.distanceTransform(1-a,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        to_b=cv2.distanceTransform(1-b,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        distances.append(float((to_a[b>0].mean()+to_b[a>0].mean())/2))
    return distances


def run(name,weight):
    model=YOLO(str(weight))
    result_dir=HERE/"results"
    result_dir.mkdir(exist_ok=True)
    # Visually identified board-only regions. Coordinates are on original 640x480 frames.
    cropdir=HERE/"challenges"
    cropdir.mkdir(exist_ok=True)
    regions=[(172,110,360,311),(330,194,370,320),(255,117,366,320),(260,114,555,324)]
    for p,(x,y,xx,yy) in zip(sorted((DATA/"images/test").glob("*.jpg")),regions):
        assert not any(m[y:yy,x:xx].any() for m in gt(p)),p
        cv2.imwrite(str(cropdir/p.name),cv2.imread(str(p))[y:yy,x:xx])
    stats={}
    detail=[]
    sheet=Image.new("RGB",(640,520))
    for group,paths in (
        ("new",sorted((DATA/"images/test").glob("*.jpg"))),
        ("old",[Path(s) for s in (OLD/"test.txt").read_text().splitlines()]),
        ("old_board_crops",sorted((BASE/"work/challenge/test").glob("*.jpg"))),
        ("new_board_crops",sorted((HERE/"challenges").glob("*.jpg"))),
        ("val",[Path(s) for s in (HERE/"val.txt").read_text().splitlines()])):
        rows=[]
        for i,p in enumerate(paths):
            predicted=model.predict(str(p),device="cpu",imgsz=384,conf=.65,retina_masks=True,verbose=False)[0]
            masks=[] if predicted.masks is None else predicted.masks.data.cpu().numpy().astype(bool)
            expected=[] if group.endswith("crops") else gt(p)
            row=score(expected,masks)
            if group=="new":
                row["edge_px"]=boundary_distances(expected,masks)
            row.update(group=group,file=p.name,confidences=predicted.boxes.conf.tolist())
            rows.append(row)
            if group=="new":
                panel=predicted.orig_img.copy()
                for items,color in [(expected,(0,235,0)),(masks,(235,0,235))]:
                    for m in items:
                        c,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(panel,c,-1,color,2)
                x,y=i%2*320,i//2*260
                sheet.paste(Image.fromarray(panel[:,:,::-1]).resize((320,240)),(x,y))
                ImageDraw.Draw(sheet).text((x+4,y+240),f"{name}  {p.name.split('_')[4]}",fill="white")
        stats[group]=aggregate(rows)
        detail.extend(rows)
    sheet.save(result_dir/(name+".jpg"),quality=95)
    index={Path(r["path"]).name:r for r in json.loads((BASE/"work/archive_index.json").read_text())}
    replay=[]
    with ZipFile(HERE.parent/"test_images.zip") as z:
        for n in z.namelist():
            p=Path(n)
            if p.name not in index: continue
            rec=index[p.name]
            raw=z.read(n)
            if digest(raw)!=digest(Path(rec["path"]).read_bytes()):
                raise ValueError("Archive image differs from indexed image")
            im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            r=model.predict(im,device="cpu",imgsz=384,conf=.65,retina_masks=True,verbose=False)[0]
            union=np.zeros(im.shape[:2],bool) if r.masks is None else r.masks.data.cpu().numpy().any(axis=0)
            overlap=0.
            for marker in rec["markers"]:
                mm=np.zeros(im.shape[:2],np.uint8)
                cv2.fillPoly(mm,[np.array(marker,np.int32)],1)
                overlap=max(overlap,(mm.astype(bool)&union).sum()/max(1,mm.sum()))
            replay.append(dict(file=n,count=len(r.boxes),markers=len(rec["markers"]),overlap=float(overlap)))
    stats["zip"]=dict(usable=len(replay),missing=sum(r["count"]==0 for r in replay),
                      multiple=sum(r["count"]>1 for r in replay),marker_frames=sum(r["markers"]>0 for r in replay),
                      spill=sum(r["overlap"]>=.5 for r in replay))
    (result_dir/(name+".json")).write_text(json.dumps(dict(stats=stats,details=detail,replay=replay),indent=2))
    print(name,json.dumps(stats))


if __name__=="__main__":
    import sys
    models={"expv3":HERE.parent/"expv3.pt","control":HERE/"runs/control/weights/best.pt",
            "expv4":HERE.parent/"expv4.pt"}
    for n in sys.argv[1:]:
        run(n,models[n])
