"""Paper-edge hard-example training. The entire November 12 clip is reserved for evaluation."""
import json
import random
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
YOLODIR=HERE.parent
BASE=YOLODIR/"board_seg"
sys.path.insert(0,str(BASE/"tools"))
# This file shares a basename with the prior trainer, so import it by path.
import importlib.util
spec=importlib.util.spec_from_file_location("board_trainer",BASE/"tools/train.py")
runtime=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
YOLO,LocalTrainer=runtime.YOLO,runtime.LocalTrainer
import cv2
import numpy as np

ARCH=YOLODIR.parent/"k210_camera/vision_grasp_images"
OLD=BASE/"work/dataset"
# Points traced at half resolution; first four are TL, TR, BR, BL of the visible silhouette.
SHAPES={
 ("20261111",8):[(139,40),(183,40),(189,183),(131,185)],
 ("20261111",30):[(140,45),(186,43),(199,192),(141,196)],
 ("20261111",53):[(116,25),(162,22),(177,162),(117,168)],
 ("20261117",24):[(189,50),(241,45),(280,205),(206,213)],
 ("20261117",129):[(167,50),(220,49),(241,218),(172,221)],
 ("20261111",13):[(139,137),(201,136),(209,240),(138,240)],
 ("20261111",22):[(138,140),(196,140),(205,240),(136,240)],
 ("20261117",33):[(129,139),(186,131),(206,240),(136,240)],
 ("20261117",92):[(110,99),(186,88),(212,240),(120,240)],
}

def save(split,name,image,mask):
    p=HERE/"data/images"/split/(name+".jpg")
    lab=HERE/"data/labels"/split/(name+".txt")
    p.parent.mkdir(exist_ok=True,parents=True);lab.parent.mkdir(exist_ok=True,parents=True)
    assert cv2.imwrite(str(p),image)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    rows=[]
    if contours:
        pts=cv2.approxPolyDP(max(contours,key=cv2.contourArea),1.2,True).reshape(-1,2)
        if len(pts)<3: raise ValueError(name)
        vals=(pts/[image.shape[1],image.shape[0]]).ravel()
        rows.append("0 "+" ".join(f"{v:.6f}" for v in vals))
    lab.write_text("\n".join(rows))
    return p

def prepare():
    rng=random.Random(421)
    train=[];val=[]
    donors={"train":[],"val":[]}
    for (day,seq),quad in SHAPES.items():
        p=next((ARCH/day).glob(f"*_seq{seq}_*.jpg"))
        im=cv2.imread(str(p))
        xy=np.float32(quad)*2
        mask=np.zeros(im.shape[:2],np.uint8)
        cv2.fillPoly(mask,[xy.astype(np.int32)],255)
        split="val" if seq in (129,92) else "train"
        (train if split=="train" else val).append(save(split,p.stem,im,mask))
        if np.max(xy[:,1])<478:
            donors[split].append((im,xy,mask))
    backgrounds=sorted((OLD/"images/paired").glob("*.jpg"))
    for split,count in (("train",80),("val",12)):
        for i in range(count):
            im,quad,m=rng.choice(donors[split])
            bg=cv2.imread(str(rng.choice(backgrounds[:-1]) if split=="train" else backgrounds[-1]))
            top=rng.uniform(155,190); bot=top+rng.uniform(210,290)
            gap=rng.uniform(0,47)
            x=546+(top-152)*.24-gap
            xb=546+(bot-152)*.24-gap+rng.uniform(-11,11)
            width=rng.uniform(79,116)
            dst=np.float32([[x-width,top+rng.uniform(-3,10)],[x,top],[xb,bot],[xb-width*1.14,bot+4]])
            H=cv2.getPerspectiveTransform(quad,dst)
            donor=cv2.warpPerspective(im,H,(640,480))
            mask=cv2.warpPerspective(m,H,(640,480),flags=cv2.INTER_NEAREST)
            if i%10:
                # Feather only the subpixel transition at the silhouette.
                a=cv2.GaussianBlur(mask,(3,3),.55).astype(np.float32)[:,:,None]/255
                bg=(bg*(1-a)+donor*a).astype(np.uint8)
            else: mask.fill(0)
            zoom=rng.uniform(1.05,2.2)
            mat=cv2.getRotationMatrix2D((x-width/2,top+60),rng.uniform(-11,11),zoom)
            mat[:,2]+=[rng.uniform(220,385)-(x-width/2),rng.uniform(165,385)-(top+60)]
            imout=cv2.warpAffine(bg,mat,(640,480),borderMode=cv2.BORDER_REPLICATE)
            mout=cv2.warpAffine(mask,mat,(640,480),flags=cv2.INTER_NEAREST)
            if i%2: imout=cv2.flip(imout,1); mout=cv2.flip(mout,1)
            (train if split=="train" else val).append(save(split,f"paper_{i:03d}",imout,mout))
    replay=[Path(s) for s in (YOLODIR/"expv4_run/control.txt").read_text().splitlines()]
    newboard=sorted((YOLODIR/"expv4_run/data/images/train").glob("photo*"))
    train+=replay+newboard
    val += [Path(s) for s in (YOLODIR/"expv4_run/val.txt").read_text().splitlines()]
    for split,paths in (("train",train),("val",val)):
        assert all(p.is_file() and "20261112_" not in p.name for p in paths)
        (HERE/f"{split}.txt").write_text("".join(str(p)+"\n" for p in paths))
    (HERE/"dataset.yaml").write_text(f"path: {HERE.as_posix()}\ntrain: train.txt\nval: val.txt\nnames: [medicine_box]\n")
    print(len(train),len(val),flush=True)

if __name__=="__main__":
    prepare()
    YOLO(str(YOLODIR/"expv4.pt")).train(trainer=LocalTrainer,data=str(HERE/"dataset.yaml"),
        project=str(HERE),name="training",epochs=100,patience=18,device="cpu",imgsz=512,
        batch=6,workers=0,freeze=10,seed=421,deterministic=True,optimizer="AdamW",lr0=.0004,
        warmup_epochs=1,mosaic=.1,close_mosaic=8,degrees=8,translate=.07,scale=.15,
        amp=False,plots=False,cache=False,verbose=False)
