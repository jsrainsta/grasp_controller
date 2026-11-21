"""Compare sizes and weights on separate validation scenes before using the reported clip."""
import json
import shutil
from pathlib import Path
import torch
from verify import HERE,t,labeled

def quality(rows):
    return sum(sum(r["ious"]) for r in rows)/max(1,sum(r["gt"]+r["fp"] for r in rows))

def main():
    val=[Path(s) for s in (HERE/"val.txt").read_text().splitlines()]
    near=[p for p in val if HERE in p.parents]
    prev=[p for p in val if p not in near]
    source=t.YOLODIR/"expv4.pt"
    trained=HERE/"training/weights/best.pt"
    state=t.YOLO(str(trained)).model.float().state_dict()
    output=t.YOLODIR/"expv4s.pt"
    trials=[];best=-1
    for f in (1.,.75,.5):
        path=trained
        if f<1:
            path=HERE/"training/weights"/f"blend_{f}.pt"
            net=t.YOLO(str(source))
            for k,v in net.model.float().state_dict().items():
                if v.is_floating_point():
                    v.lerp_(state[k],f)
                else: v.copy_(state[k])
            net.save(str(path))
        model=t.YOLO(str(path))
        for size in (384,512,640):
            hard=labeled(model,near,size);general=labeled(model,prev,size)
            ring=sum(r["ring"] for r in hard if r["gt"])/sum(r["gt"]>0 for r in hard)
            metric=.5*quality(hard)+.3*quality(general)+.2*(1-ring)
            trials.append(dict(fraction=f,size=size,confidence=.65,hard=quality(hard),
                general=quality(general),ring=ring,score=metric))
            if metric>best:
                shutil.copyfile(path,output)
                best=metric
    selected=max(trials,key=lambda r:r["score"])
    (HERE/"settings.json").write_text(json.dumps(dict(selected=selected,validation=trials),indent=2))
    print(selected)

if __name__=="__main__":
    with torch.no_grad():main()
