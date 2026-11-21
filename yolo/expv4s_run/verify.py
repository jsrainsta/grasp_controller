"""Compare masks on the reported strip, acquisition clips and labeled regression images."""
import csv
import hashlib
import json
from pathlib import Path
import sys
from zipfile import ZipFile
import importlib.util

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("paper_train",HERE/"train.py")
t=importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
import cv2
import numpy as np
from PIL import Image,ImageDraw
from evaluate import score,aggregate

# Manually located inner/outer paper edges at two scanlines (320x240 reference).
# Interpolate *inside* the visible strip, excluding its uncertain boundary pixels.
EDGES=[
    (50,226,233,110,239,249),(155,189,204,195,199,216),(155,188,204,195,199,216),
    (132,193,210,180,205,224),(95,197,215,165,215,236),(95,197,214,165,212,235),
    (95,198,214,165,212,235),(95,194,208,165,206,229),(95,194,210,165,208,230),
    (95,196,213,165,210,233)]
SPANS=[(40,128),(148,205),(148,205),(124,198),(89,198),(89,196),(80,190),(84,194),(86,194),(73,191)]
def probes(seq):
    a,b,c,d,e,f=EDGES[seq-104]
    top,bottom=SPANS[seq-104]
    yy,xx=np.mgrid[:480,:640]/2.
    pos=(yy-a)/(d-a)
    inner=b+pos*(e-b);outer=c+pos*(f-c)
    band=(yy>=top)&(yy<=bottom)
    return band&(xx>=inner+2)&(xx<=outer-2),band&(xx>=inner-12)&(xx<=inner-3)

def predict(model,im,size):
    res=model.predict(im,imgsz=size,conf=.65,device="cpu",retina_masks=True,verbose=False)[0]
    masks=[] if res.masks is None else res.masks.data.cpu().numpy().astype(bool)
    return masks

def labeled(model,paths,size):
    rows=[]
    for p in paths:
        p=Path(p);im=cv2.imread(str(p));h,w=im.shape[:2]
        targets=[]
        lab=p.parents[2]/"labels"/p.parent.name/(p.stem+".txt")
        for s in lab.read_text().splitlines():
            q=np.array(list(map(float,s.split()[1:]))).reshape(-1,2)*[w,h]
            mask=np.zeros((h,w),np.uint8);cv2.fillPoly(mask,[q.round().astype(np.int32)],1);targets.append(mask.astype(bool))
        masks=predict(model,im,size)
        row=score(targets,masks)
        # Count pixels spilling immediately outside each reference silhouette.
        gt=np.logical_or.reduce(targets) if targets else np.zeros((h,w),bool)
        pred=np.logical_or.reduce(masks) if len(masks) else np.zeros_like(gt)
        ring=cv2.dilate(gt.astype('uint8'),np.ones((23,23),np.uint8)).astype(bool)&~gt
        row.update(file=p.name,ring=float(pred[ring].mean()) if ring.any() else 0.)
        rows.append(row)
    return rows

def run(name,model,size):
    root=HERE/"evaluation";root.mkdir(exist_ok=True)
    records=[];tiles=[];packed={}
    issues=next(p for p in (t.YOLODIR/"medicine_results_v4").iterdir() if p.is_dir() and "问题" in p.name)
    selected={p.name for p in issues.glob("*.jpg")}
    with ZipFile(t.YOLODIR/"test_images.zip") as z:
        names=sorted(z.namelist())
        assert selected<=set(names) and len(selected)==10
        for n in names:
            raw=z.read(n);assert raw==(t.ARCH/"20261112"/n).read_bytes()
            im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            masks=predict(model,im,size)
            union=np.logical_or.reduce(masks) if len(masks) else np.zeros(im.shape[:2],bool)
            packed[n]=np.packbits(union)
            row=dict(file=n,count=len(masks),bad=n in selected)
            if n in selected:
                seq=int(n.split("_")[4][3:]);paper,box=probes(seq)
                row.update(paper=float(union[paper].mean()),box=float(union[box].mean()))
                original=cv2.imdecode(np.frombuffer((issues/n).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
                # Only for diagnosing the supplied overlays. Labels and rectangle borders can also be blue.
                delta=original.astype(float)-im.astype(float)
                painted=(delta[:,:,0]-delta[:,:,2]>40)&(delta[:,:,2]<-12)
                row["submitted_blue"]=float(painted[paper].mean())
                view=im.copy()
                view[union]=(view[union].astype(float)*.57+[100,0,70]).clip(0,255).astype(np.uint8)
                for mask,col in ((paper,(0,215,255)),(box,(0,200,0))):
                    cv2.drawContours(view,cv2.findContours(mask.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],-1,col,1)
                tiles.append((seq,view))
            records.append(row)
    sheet=Image.new('RGB',(640,5*262))
    for i,(seq,v) in enumerate(sorted(tiles,key=lambda v:v[0])):
        x=i%2*320;y=i//2*262;sheet.paste(Image.fromarray(v[:,:,::-1]).resize((320,240)),(x,y))
        ImageDraw.Draw(sheet).text((x+4,y+241),f"{name}   seq{seq}",fill="white")
    sheet.save(root/(name+".jpg"),quality=94)
    groups={"old":(t.OLD/"test.txt").read_text().splitlines(),
            "newboard":sorted((t.YOLODIR/"expv4_run/data/images/test").glob("*.jpg"))}
    summary={k:aggregate(labeled(model,ps,size)) for k,ps in groups.items()}
    flagged=[r for r in records if r["bad"]]
    summary["issues"]=dict(mean_paper=float(np.mean([r["paper"] for r in flagged])),
        max_paper=max(r["paper"] for r in flagged),above5=sum(r["paper"]>.05 for r in flagged),
        mean_box=float(np.mean([r["box"] for r in flagged])),one=sum(r["count"]==1 for r in flagged))
    summary["other103"]=dict(one=sum(r["count"]==1 for r in records if not r["bad"]))
    (root/(name+".json")).write_text(json.dumps(dict(size=size,summary=summary,images=records),indent=2))
    np.savez_compressed(root/(name+"_masks.npz"),**packed)
    print(name,summary,flush=True)

if __name__=="__main__":
    weights=Path(sys.argv[2])
    run(sys.argv[1],t.YOLO(str(weights)),int(sys.argv[3]))
