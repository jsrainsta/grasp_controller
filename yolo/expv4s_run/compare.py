"""Build a per-frame visual comparison and a CSV of paper/box coverage."""
import csv
import json
from pathlib import Path
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
OUT=HERE/"evaluation"
def main():
    before=json.loads((OUT/"expv4_384.json").read_text())
    after=json.loads((OUT/"expv4s.json").read_text())
    old={r["file"]:r for r in before["images"] if r["bad"]}
    new={r["file"]:r for r in after["images"] if r["bad"]}
    submitted=HERE.parent/"medicine_results_v4"/"存在问题的"
    orig=HERE.parents[1]/"k210_camera/vision_grasp_images/20261112"
    panel=Image.new("RGB",(960,10*262))
    new_sheet=Image.open(OUT/"expv4s.jpg")
    csvrows=[]
    for i,n in enumerate(sorted(old)):
        seq=n.split("_")[4]
        a=old[n];b=new[n]
        x=i%2*320;y=i//2*262
        views=[Image.open(orig/n).resize((320,240)),Image.open(submitted/n).resize((320,240)),
               new_sheet.crop((x,y,x+320,y+240))]
        for j,(im,label) in enumerate(zip(views,("Original","Submitted expv4","expv4s + reference strips"))):
            panel.paste(im,(j*320,i*262))
            ImageDraw.Draw(panel).text((j*320+4,i*262+241),seq+"   "+label,fill="white")
        csvrows.append(dict(file=n,old_paper=a["paper"],new_paper=b["paper"],old_box=a["box"],
                            new_box=b["box"],old_instances=a["count"],new_instances=b["count"]))
    panel.save(OUT/"reported_10_comparison.jpg",quality=95)
    with (OUT/"reported_10.csv").open("w",encoding="utf-8-sig",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(csvrows[0]));writer.writeheader();writer.writerows(csvrows)

if __name__=="__main__":main()
