"""Preview expv4s with the input size selected during verification."""
import argparse
import json
from verify import HERE,t

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("source",help="Path to an original photo or a folder of photos")
    args=parser.parse_args()
    cfg=json.loads((HERE/"settings.json").read_text())["selected"]
    model=t.YOLO(str(t.YOLODIR/"expv4s.pt"))
    for r in model.predict(args.source,device="cpu",imgsz=cfg["size"],conf=cfg["confidence"],
                           retina_masks=True,save=True,project=str(HERE),name="previews",stream=True,verbose=False):
        print(r.path,len(r.boxes),r.save_dir)
