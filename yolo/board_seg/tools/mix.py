"""Refine public quadrilateral annotations into visible contours; compose training-only board scenes."""
import json
import random

import cv2
import numpy as np
from PIL import Image, ImageDraw

from prepare import DATA, ROOT, WORK, write_item


def refine(image, row):
    h, w = image.shape[:2]
    poly = (np.array(row).reshape(-1, 2) * [w, h]).astype(np.int32)
    x,y,bw,bh = cv2.boundingRect(poly)
    pad = max(8, int(max(bw,bh)*.3))
    xx, yy = max(0,x-pad), max(0,y-pad)
    roi = image[yy:y+bh+pad, xx:x+bw+pad]
    initial = np.zeros(roi.shape[:2], np.uint8)
    cv2.fillPoly(initial, [poly - [xx,yy]], 1)
    # Keep confident central pixels; allow boundaries to move to the actual side faces.
    labels = np.full(initial.shape, cv2.GC_PR_BGD, np.uint8)
    labels[initial>0] = cv2.GC_PR_FGD
    kernel = np.ones((max(3,min(bw,bh)//3),)*2, np.uint8)
    labels[cv2.erode(initial,kernel)>0] = cv2.GC_FGD
    labels[[0,-1],:] = cv2.GC_BGD
    labels[:,[0,-1]] = cv2.GC_BGD
    if not np.any(labels==cv2.GC_FGD):
        return None
    cv2.grabCut(roi,labels,None,np.zeros((1,65)),np.zeros((1,65)),4,cv2.GC_INIT_WITH_MASK)
    mask = (labels%2).astype(np.uint8)
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours,key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < 300 or not .72 < area / initial.sum() < 1.35:
        return None
    contour = cv2.approxPolyDP(contour, max(.8,cv2.arcLength(contour,True)*.004),True).reshape(-1,2)
    return contour + [xx,yy] if len(contour)>=3 else None


def board_scene(image, contour, rng):
    aruco = cv2.aruco
    board = aruco.CharucoBoard((7,5),1.,.68,aruco.getPredefinedDictionary(aruco.DICT_4X4_50))
    board_img = cv2.copyMakeBorder(board.generateImage((350,250)),20,20,20,20,cv2.BORDER_CONSTANT,value=255)
    board_img = cv2.cvtColor(board_img,cv2.COLOR_GRAY2BGR)
    board_img = (board_img.astype(float)*rng.uniform(.57,.84)+rng.randint(7,27)).astype(np.uint8)
    src = np.float32([(0,0),(389,0),(389,289),(0,289)])
    dst = np.float32([(rng.randint(35,130),rng.randint(20,115)),(rng.randint(440,585),rng.randint(35,115)),
                      (rng.randint(470,635),rng.randint(345,465)),(rng.randint(0,110),rng.randint(335,460))])
    H = cv2.getPerspectiveTransform(src,dst)
    scene = cv2.warpPerspective(board_img,H,(640,480),borderValue=(95,155,183))
    x,y,w,h = cv2.boundingRect(contour.astype(np.int32))
    scale = min(rng.uniform(130,260)/max(w,h), 4)
    crop=cv2.resize(image[y:y+h,x:x+w],None,fx=scale,fy=scale)
    ox,oy=rng.randint(95,320),rng.randint(95,205)
    q=np.round((contour-[x,y])*scale).astype(np.int32)
    m=np.zeros(crop.shape[:2],np.uint8)
    cv2.fillPoly(m,[q],255)
    roi=scene[oy:oy+crop.shape[0],ox:ox+crop.shape[1]]
    np.copyto(roi,crop,where=m[:,:,None].astype(bool))
    return cv2.GaussianBlur(scene,(3,3),.45), (q+[ox,oy]) / [640,480]


def main():
    rng=random.Random(261119)
    cv2.setRNGSeed(26)
    public=WORK/"public"
    fs=sorted(public.glob("*.jpg"))
    audit=Image.new("RGB",(800, ((len(fs)+7)//8)*88))
    draw=ImageDraw.Draw(audit)
    included=[]
    exclusions=ROOT/"tools/public_exclusions.txt"
    excluded=set(exclusions.read_text().split()) if exclusions.exists() else set()
    for i,p in enumerate(fs):
        image=cv2.imread(str(p))
        h,w=image.shape[:2]
        rows=[list(map(float,s.split()[1:])) for s in p.with_suffix(".txt").read_text().splitlines()]
        # Reject full samples if any instance fails. Crowded scenes with missed neighbours
        # would teach false background. Use the clearly separated single-box photos.
        if len(rows)!=1 or p.stem in excluded:
            continue
        poly=refine(image,rows[0])
        if poly is None:
            continue
        q=(poly/[w,h]).clip(0,1)
        written=write_item("public","ext_"+p.stem,p,[q.ravel()])
        included.append(written)
        if len(included)%3==1:
            scene,points=board_scene(image,poly,rng)
            included.append(write_item("public","render_"+p.stem,scene,[points.ravel()]))
        x,y,bw,bh=cv2.boundingRect(poly.astype(np.int32))
        cv2.polylines(image,[poly.astype(np.int32)],True,(0,255,0),1)
        roi=image[max(0,y-7):y+bh+7,max(0,x-7):x+bw+7]
        im=Image.fromarray(cv2.cvtColor(roi,cv2.COLOR_BGR2RGB))
        im.thumbnail((99,70))
        ox,oy=i%8*100,i//8*88
        audit.paste(im,(ox,oy))
        draw.text((ox,oy+70),str(i),fill="white")
    audit.save(WORK/"public_review.jpg")
    local=(DATA/"train.txt").read_text(encoding="utf8")
    (DATA/"mixed.txt").write_text(local+"".join(str(p)+"\n" for p in included),encoding="utf8")
    print(len(included),"external and composed segmentation samples")


if __name__=="__main__":
    main()
