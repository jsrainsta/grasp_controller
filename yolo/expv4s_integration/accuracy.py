"""Score visible instances and compare both top-face estimates with hand-picked image landmarks."""
import argparse
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
from PIL import Image, ImageDraw

from replay import HERE, ARM, OCRRunner, load, save
from box_segmentation import image_masks, overview_mask

# Approximate visible rim corners on raw K210 photographs, in original pixels.
RIMS = {
    4: [(287,29),(372,29),(381,267),(272,266)],
    33: [(291,28),(375,28),(383,258),(276,260)],
    45: [(296,27),(382,28),(383,267),(273,266)],
    104: [(359,57),(443,54),(488,290),(382,297)],
}


def middle(points):
    a,b,c,d=np.column_stack((points,np.ones(4)))
    intersect=np.cross(np.cross(a,c),np.cross(b,d))
    return intersect[:2]/intersect[2]


def evaluate_instances():
    root=ARM.parent/"yolo/dataset"
    rows=[]
    canvas=Image.new("RGB",(5*240,4*202))
    runner=OCRRunner(target_id="med_003")
    with TemporaryDirectory() as temp:
        for line in (root/"medicine-box-allsides.ndjson").read_text(encoding="utf8").splitlines():
            record=json.loads(line)
            if record.get("split")!="test":
                continue
            p=root/"compressed_640x480"/record["file"]
            im=cv2.imdecode(np.fromfile(p,np.uint8),1)
            h,w=im.shape[:2]
            masks=image_masks(p)
            gt=[]
            for seg in record.get("annotations",{}).get("segments",[]):
                m=np.zeros((h,w),np.uint8)
                cv2.fillPoly(m,[np.rint(np.reshape(seg[1:],(-1,2))*[w,h]).astype(np.int32)],255)
                gt.append(m)
            scores=sorted(((np.count_nonzero(a&b)/max(1,np.count_nonzero(a|b)),i,j)
                          for i,a in enumerate(gt) for j,b in enumerate(masks)),reverse=True)
            matched_gt=set();matched_pred=set();iou=[]
            for score,i,j in scores:
                if score>=.5 and i not in matched_gt and j not in matched_pred:
                    matched_gt.add(i);matched_pred.add(j);iou.append(score)
            ocr=runner.run(p,Path(temp),known_box_focus=True)
            associated=overview_mask(masks,(h,w),ocr)
            for mask in gt:
                cv2.drawContours(im,cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],-1,(0,200,0),2)
            for mask in masks:
                cv2.drawContours(im,cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],-1,(255,0,220),2)
            x=len(rows)%5*240;y=len(rows)//5*202
            canvas.paste(Image.fromarray(im[:,:,::-1]).resize((240,180)),(x,y))
            ImageDraw.Draw(canvas).text((x+2,y+182),f"{p.stem[-22:]} GT {len(gt)}  YOLO {len(masks)}",fill="white")
            rows.append(dict(file=p.name,labels=len(gt),pred=len(masks),ious=iou,
                             target_status=ocr["target_selection"]["status"],associated=associated is not None))
    canvas.save(HERE/"instances.jpg",quality=93)
    save(HERE/"instances.json",rows)


def evaluate_rims():
    canvas=Image.new("RGB",(640,220*len(RIMS)))
    scores=[]
    for case in load(HERE/"replay_results.json"):
        name=case["legacy"]["image"]
        seq=int(name.split("_")[4][3:])
        if seq not in RIMS:continue
        frame=cv2.imdecode(np.fromfile(ARM/"vision_grasp_images/20261112"/name,np.uint8),1)
        hand=np.array(RIMS[seq],np.float64)
        cv2.polylines(frame,[hand.astype(np.int32)],True,(0,200,0),2)
        score={"image":name,"manual_reference":hand.tolist()}
        masks=image_masks(ARM/"vision_grasp_images/20261112"/name)
        if masks is not None and len(masks)==1:
            ys,xs=np.nonzero(masks[0])
            centroid=np.array([xs.mean(),ys.mean()])
            score["whole_mask_center_error_px"]=float(np.linalg.norm(centroid-middle(hand)))
            cv2.drawMarker(frame,np.rint(centroid).astype(int),(255,210,0),cv2.MARKER_TILTED_CROSS,12,1)
        for key,color in (("legacy",(0,170,255)),("fusion",(255,0,220))):
            if "corners" not in case[key]:continue
            q=np.array(case[key]["corners"])
            score[key]=float(np.linalg.norm(middle(q)-middle(hand)))
            cv2.polylines(frame,[np.rint(q).astype(np.int32)],True,color,1)
            cv2.drawMarker(frame,np.rint(middle(q)).astype(int),color,cv2.MARKER_CROSS,10,1)
        cv2.drawMarker(frame,np.rint(middle(hand)).astype(int),(0,200,0),cv2.MARKER_CROSS,12,1)
        y=len(scores)*220
        canvas.paste(Image.fromarray(frame[:,:,::-1]).resize((293,220)),(0,y))
        cx,cy=np.rint(middle(hand)).astype(int)
        crop=frame[max(0,cy-160):cy+160,cx-110:cx+110]
        view=Image.fromarray(crop[:,:,::-1]);view.thumbnail((180,217))
        canvas.paste(view,(315,y))
        ImageDraw.Draw(canvas).text((500,y+100),f"seq{seq}",fill="white")
        scores.append(score)
    canvas.save(HERE/"top_faces.jpg",quality=95)
    save(HERE/"top_faces.json",scores)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--interpreter",required=True)
    parser.add_argument("--rims",action="store_true")
    args=parser.parse_args()
    os.environ["GRASP_YOLO_PYTHON"]=args.interpreter
    if args.rims:
        evaluate_rims()
    else:
        evaluate_instances()
