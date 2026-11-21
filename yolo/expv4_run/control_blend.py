"""Apply the published interpolation fraction to the old-data-only control."""
import json
import torch
from check import HERE, YOLO, run

if __name__=="__main__":
    choices=json.loads((HERE/"selection.json").read_text())
    alpha=max((c for c in choices if c["alpha"]),key=lambda c:c["score"])["alpha"]
    model=YOLO(str(HERE.parent/"expv3.pt"))
    other=YOLO(str(HERE/"runs/control/weights/best.pt")).model.float().state_dict()
    with torch.no_grad():
        for key,value in model.model.float().state_dict().items():
            if value.is_floating_point():
                value.lerp_(other[key],alpha)
            else:
                value.copy_(other[key])
    path=HERE/"runs/control/weights/blended.pt"
    model.save(str(path))
    run("control_blend",path)
