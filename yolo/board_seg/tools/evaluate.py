"""Choose the operating point on validation only, then score untouched robot images."""
import argparse
import json
import statistics
import csv
from pathlib import Path

from train import ROOT, YOLO
import cv2
import numpy as np

DATA = ROOT/"work/dataset"
OUT = ROOT/"evaluation"
MODELS = {"exp": Path.home()/"Downloads/exp.pt", "expv2": ROOT.parent/"expv2.pt",
          **{name: ROOT/"runs"/name/"weights/best.pt" for name in ("local","mixed","paired")}}
SIZES = (384, 640)
THRESHOLDS = (.25, .45, .65)


def mask_targets(path, hw, crop):
    if crop:
        return []
    label = DATA/"labels"/path.parent.name/(path.stem+".txt")
    h,w = hw
    masks=[]
    for row in label.read_text().splitlines():
        xy = (np.array(list(map(float,row.split()[1:]))).reshape(-1,2)*[w,h]).round().astype(np.int32)
        m=np.zeros((h,w),np.uint8)
        cv2.fillPoly(m,[xy],1)
        masks.append(m.astype(bool))
    return masks


def score(gts, masks):
    matches=[]
    for i,m in enumerate(masks):
        for j,gt in enumerate(gts):
            hit=int(np.count_nonzero(m & gt))
            iou=hit/max(1,int(np.count_nonzero(m | gt)))
            if iou>=.5:
                matches.append((iou,i,j,1-hit/max(1,int(m.sum()))))
    pm,gm=set(),set()
    ious,spills=[],[]
    for iou,i,j,spill in sorted(matches,reverse=True):
        if i not in pm and j not in gm:
            pm.add(i)
            gm.add(j)
            ious.append(iou)
            spills.append(spill)
    return dict(gt=len(gts),tp=len(ious),fp=len(masks)-len(ious),fn=len(gts)-len(ious),
                ious=ious,spills=spills)


def aggregate(rows):
    tp,fp,fn=[sum(r[key] for r in rows) for key in ("tp","fp","fn")]
    ious=[x for r in rows for x in r["ious"]]
    spills=[x for r in rows for x in r["spills"]]
    return dict(images=len(rows),tp=tp,fp=fp,fn=fn,f1=2*tp/max(1,2*tp+fp+fn),
                mean_iou=statistics.mean(ious) if ious else 0,
                mean_spill=statistics.mean(spills) if spills else 0,
                false_alarm_images=sum(r["gt"]==0 and r["fp"]>0 for r in rows))


def evaluate(name, split, sizes, thresholds, save_overlays=False):
    model=YOLO(str(MODELS[name]))
    normal=[Path(s) for s in (DATA/(split+".txt")).read_text().splitlines()]
    challenge=sorted((ROOT/"work/challenge"/split).glob("*.jpg"))
    allrows=[]
    for size in sizes:
        for p in normal+challenge:
            crop=p in challenge
            result=model.predict(str(p), device="cpu",imgsz=size,conf=min(thresholds),retina_masks=True,
                                 verbose=False)[0]
            masks=result.masks.data.cpu().numpy().astype(bool) if result.masks is not None else []
            conf=result.boxes.conf.cpu().tolist()
            targets=mask_targets(p,result.orig_shape,crop)
            for threshold in thresholds:
                kept=[m for m,c in zip(masks,conf) if c>=threshold]
                row=score(targets,kept)
                group="board_crop" if crop else ("full_board" if p.name.startswith("photo_") else "phone")
                allrows.append(dict(name=name,size=size,conf=threshold,group=group,file=p.name,**row))
                if save_overlays and not crop:
                    panel=result.orig_img.copy()
                    for k,color in [(targets,(0,225,0)),(kept,(235,35,235))]:
                        for mask in k:
                            contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                            cv2.drawContours(panel,contours,-1,color,2)
                    directory=OUT/"overlays"/name
                    directory.mkdir(exist_ok=True,parents=True)
                    cv2.imwrite(str(directory/p.name),panel)
    return allrows


def validation_choice(rows):
    candidates=[]
    for size in SIZES:
        for conf in THRESHOLDS:
            filtered=[r for r in rows if r["size"]==size and r["conf"]==conf]
            allstats=aggregate(filtered)
            board=aggregate([r for r in filtered if r["group"]!="phone"])
            quality=(allstats["f1"]*.5+board["f1"]*.25+board["mean_iou"]*.25)
            candidates.append(dict(score=quality,size=size,conf=conf,all=allstats,board=board))
    return max(candidates,key=lambda c:(c["score"],-c["size"],c["conf"]))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("stage",choices=("val","test"))
    parser.add_argument("names",nargs="*",default=list(MODELS))
    args=parser.parse_args()
    OUT.mkdir(exist_ok=True)
    if args.stage=="val":
        for name in args.names:
            rows=evaluate(name,"val",SIZES,THRESHOLDS)
            info=dict(choice=validation_choice(rows),rows=rows)
            (OUT/(name+"_val.json")).write_text(json.dumps(info),encoding="utf8")
            print(name, info["choice"],flush=True)
        return
    choices={n:json.loads((OUT/(n+"_val.json")).read_text())["choice"] for n in args.names}
    new_choice=max(("local","mixed","paired"),key=lambda n:choices[n]["score"])
    selection=dict(new_choice=new_choice,validation=choices)
    (OUT/"selection.json").write_text(json.dumps(selection,indent=2),encoding="utf8")
    summary={}
    common={}
    records=[]
    for name,ch in choices.items():
        rows=evaluate(name,"test",[ch["size"]],[ch["conf"]],True)
        groups={"full_positive":[r for r in rows if r["group"]=="full_board" and r["gt"]],
                "full_empty":[r for r in rows if r["group"]=="full_board" and not r["gt"]],
                "board_crops":[r for r in rows if r["group"]=="board_crop"]}
        summary[name]={k:aggregate(v) for k,v in groups.items()}
        fixed=evaluate(name,"test",[640],[.25])
        common[name]={
            "full_positive":aggregate([r for r in fixed if r["group"]=="full_board" and r["gt"]]),
            "full_empty":aggregate([r for r in fixed if r["group"]=="full_board" and not r["gt"]]),
            "board_crops":aggregate([r for r in fixed if r["group"]=="board_crop"])}
        records.extend(rows)
        print(name,summary[name],flush=True)
    (OUT/"test_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf8")
    (OUT/"test_common_640_conf025.json").write_text(json.dumps(common,indent=2),encoding="utf8")
    with (OUT/"test_per_image.csv").open("w",newline="",encoding="utf-8-sig") as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


if __name__=="__main__":
    main()
