"""Prepare grouped edge scenes and fine-tune v3 with and without the new data."""
from pathlib import Path
import random
import shutil
import json
import sys

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/"board_seg"
sys.path.insert(0,str(BASE/"tools"))
from train import LocalTrainer,YOLO
import cv2
import numpy as np

DATA=HERE/"data"
OLD=BASE/"work/dataset"

def save(split,name,image,rows):
    p=DATA/"images"/split/(name+".jpg")
    label=DATA/"labels"/split/(name+".txt")
    p.parent.mkdir(exist_ok=True,parents=True)
    label.parent.mkdir(exist_ok=True,parents=True)
    if isinstance(image,Path):
        shutil.copyfile(image,p)
    elif not cv2.imwrite(str(p),image):
        raise IOError(p)
    label.write_text("".join("0 "+" ".join(f"{n:.6f}" for n in q)+"\n" for q in rows))
    return p

def prepare():
    rng=random.Random(1121)
    records=[json.loads(s) for s in next((HERE.parent/"dataset").glob("*.ndjson")).read_text(encoding="utf8").splitlines()]
    new=sorted((r for r in records if r.get("file","").startswith("photo_20261120")),key=lambda r:r["file"])
    assert len(new)==11
    backgrounds=[]
    train=[]
    for i,r in enumerate(new,1):
        path=HERE.parent/"dataset/compressed_640x480"/r["file"]
        if i==1: # Another partially visible carton at bottom has no annotation.
            continue
        split="test" if i>=8 else "val" if i==2 else "train"
        rows=[obj[1:] for obj in r["annotations"]["segments"]]
        p=save(split,path.stem,path,rows)
        if split=="train":
            train.append(p)
            im=cv2.imread(str(p))
            x=int(min(rows[0][::2])*im.shape[1])-12
            backgrounds.append(im[:,:x].copy())
    allold=[Path(s) for s in (OLD/"mixed.txt").read_text().splitlines()]
    # The checkpoint already learned all 244 images. Replay a fixed stratified subset.
    replay=[]
    for prefix,count in [("IMG_",46),("photo_",12),("boardcrop_",6),("ext_",20),("render_",10)]:
        pool=sorted(p for p in allold if p.name.startswith(prefix))
        replay+=rng.sample(pool,min(count,len(pool)))
    sources=[p for p in allold if p.name.startswith(("ext_","IMG_"))]
    for n in range(45):
        bg=backgrounds[n%len(backgrounds)].copy()
        h,w=bg.shape[:2]
        if n%6==0:
            train.append(save("train",f"blank_{n}",bg,[]))
            continue
        for _ in range(100):
            src=rng.choice(sources)
            ann=(OLD/"labels"/src.parent.name/(src.stem+".txt")).read_text().splitlines()
            if not ann: continue
            im=cv2.imread(str(src)); sh,sw=im.shape[:2]
            q=(np.array(list(map(float,rng.choice(ann).split()[1:]))).reshape(-1,2)*[sw,sh]).astype("int32")
            x,y,bw,bh=cv2.boundingRect(q)
            if min(bw,bh)>20 and x>1 and y>1 and x+bw<sw-1 and y+bh<sh-1: break
        else: raise ValueError("Missing intact donor")
        scale=rng.uniform(100,205)/max(bw,bh)
        box=cv2.resize(im[y:y+bh,x:x+bw],None,fx=scale,fy=scale)
        hh,ww=box.shape[:2]
        # Across the top, left or bottom boundary of the printed board.
        ox=rng.randint(2,max(2,w-ww-2)); oy=rng.randint(2,max(2,h-hh-2))
        if n%3==0: ox=rng.randint(2,45)
        elif n%3==1: oy=max(2,95-hh//2+rng.randint(-20,20))
        else: oy=min(h-hh-2,365-hh//2+rng.randint(-20,20))
        pts=np.round((q-[x,y])*scale).astype("int32")
        mask=np.zeros((hh,ww),np.uint8);cv2.fillPoly(mask,[pts],1)
        np.copyto(bg[oy:oy+hh,ox:ox+ww],box,where=mask[:,:,None].astype(bool))
        train.append(save("train",f"paste_{n}",bg,[((pts+[ox,oy])/[w,h]).ravel()]))
    val=[Path(s) for s in (OLD/"val.txt").read_text().splitlines()]+sorted((DATA/"images/val").glob("*.jpg"))
    for key,items in dict(control=replay,edge=replay+train,val=val).items():
        (HERE/(key+".txt")).write_text("".join(str(p)+"\n" for p in items),encoding="utf8")
    for key in ["control","edge"]:
        (HERE/(key+".yaml")).write_text(f"path: {HERE.as_posix()}\ntrain: {key}.txt\nval: val.txt\nnames: [medicine_box]\n")
    print("control, edge, validation",len(replay),len(replay+train),len(val))

if __name__=="__main__":
    key=sys.argv[1]
    if key=="prepare":
        prepare()
    elif key in ("control","edge"):
        YOLO(str(HERE.parent/"expv3.pt")).train(trainer=LocalTrainer,data=str(HERE/(key+".yaml")),
            project=str(HERE/"runs"),name=key,device="cpu",epochs=100,patience=20,
            imgsz=384,batch=8,workers=0,freeze=10,seed=26,deterministic=True,optimizer="AdamW",
            lr0=.00035,warmup_epochs=2,mosaic=.15,close_mosaic=10,degrees=10,translate=.1,
            scale=.2,amp=False,plots=False,cache=False,verbose=False)
