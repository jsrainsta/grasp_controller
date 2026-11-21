"""Run the published expv4 on saved images with the evaluated settings."""
import argparse
from experiment import HERE,YOLO

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("--variant",choices=("expv4","long"),default="expv4")
    args=parser.parse_args()
    weights=HERE.parent/"expv4.pt" if args.variant=="expv4" else HERE/"runs/control/weights/best.pt"
    predictor=YOLO(str(weights))
    for result in predictor.predict(args.source,stream=True,device="cpu",imgsz=384,conf=.65,
                                    retina_masks=True,save=True,project=str(HERE/"previews"),
                                    name=args.variant,verbose=False):
        print(result.path,len(result.boxes),result.save_dir)
