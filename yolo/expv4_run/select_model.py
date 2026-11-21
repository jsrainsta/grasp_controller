"""Select a transfer checkpoint on validation scenes before opening the new test split."""
import json
import shutil
from pathlib import Path

from check import HERE,YOLO,gt,score
import torch


def quality(rows):
    # Unmatched boxes get zero overlap; extra predictions are penalized too.
    return sum(sum(r["ious"]) for r in rows)/max(1,sum(r["gt"]+r["fp"] for r in rows))


def main():
    old=HERE.parent/"expv3.pt"
    new=HERE/"runs/edge/weights/best.pt"
    inputs=[Path(s) for s in (HERE/"val.txt").read_text().splitlines()]
    new_state=YOLO(str(new)).model.float().state_dict()
    candidates=[]
    output=HERE.parent/"expv4.pt"
    temp=HERE/"candidate.pt"
    for alpha in (0.,.25,.5,.75,1.):
        if alpha==1.:
            model=YOLO(str(new))
        else:
            model=YOLO(str(old))
            if alpha:
                state=model.model.float().state_dict()
                assert state.keys()==new_state.keys()
                for k,value in state.items():
                    if value.is_floating_point():
                        value.lerp_(new_state[k],alpha)
                    else:
                        value.copy_(new_state[k])
                model.save(str(temp))
                model=YOLO(str(temp))
        rows=[]
        for p in inputs:
            r=model.predict(str(p),imgsz=384,conf=.65,device="cpu",retina_masks=True,verbose=False)[0]
            masks=[] if r.masks is None else r.masks.data.cpu().numpy().astype(bool)
            rows.append(score(gt(p),masks))
        oldscore=quality(rows[:-1]);edge=quality(rows[-1:])
        candidates.append(dict(alpha=alpha,old=oldscore,edge=edge,score=.7*oldscore+.3*edge))
        if alpha and candidates[-1]["score"]==max(c["score"] for c in candidates if c["alpha"]):
            shutil.copyfile(new if alpha==1. else temp,output)
    if temp.exists(): temp.unlink()
    (HERE/"selection.json").write_text(json.dumps(candidates,indent=2))
    print(candidates)


if __name__=="__main__":
    with torch.no_grad():
        main()
