"""Old-day camera replay, additional manually outlined frames, and shape changes on other zip frames."""
import json
import csv
import numpy as np
import cv2
from PIL import Image,ImageDraw
from zipfile import ZipFile
from verify import HERE,t,score,aggregate,predict

OUT=HERE/"evaluation"
SHAPES={
3:[(126,239),(126,149),(129,131),(182,131),(192,239)],
11:[(119,239),(142,149),(148,138),(205,153),(188,239)],
19:[(120,239),(145,153),(151,141),(207,156),(191,239)],
27:[(17,195),(29,79),(34,52),(80,49),(91,68),(88,197),(80,205),(22,204)],
32:[(116,239),(141,160),(150,142),(206,156),(187,239)],
48:[(2,181),(16,64),(22,42),(69,39),(80,58),(74,184),(66,190),(3,189)],
}

def main():
    settings=json.loads((HERE/"settings.json").read_text())["selected"]
    clean={r["path"] for r in json.loads((t.BASE/"work/archive_index.json").read_text())}
    summary={}
    for name,size in [("expv4",384),("expv4s",settings["size"])]:
        model=t.YOLO(str(t.YOLODIR/(name+".pt")))
        rows=[];counts=[];excluded=[]
        for p in sorted((t.ARCH/"20261110").glob("*.jpg")):
            if str(p) not in clean:
                excluded.append(p.name);continue
            im=cv2.imread(str(p));masks=predict(model,im,size)
            counts.append(len(masks))
            seq=int(p.name.split("_")[4][3:])
            if seq in SHAPES:
                m=np.zeros(im.shape[:2],np.uint8)
                cv2.fillPoly(m,[np.array(SHAPES[seq],np.int32)*2],1)
                q=score([m.astype(bool)],masks)
                q["file"]=p.name;rows.append(q)
        summary[name]=dict(valid=len(counts),excluded=excluded,one=counts.count(1),
                           manual=aggregate(rows),rows=rows)
    records=[]
    with np.load(OUT/"expv4_384_masks.npz") as before,np.load(OUT/"expv4s_masks.npz") as after:
        for n in before.files:
            if int(n.split("_")[4][3:])>=104:continue
            a=np.unpackbits(before[n]).astype(bool);b=np.unpackbits(after[n]).astype(bool)
            records.append((float((a&b).sum()/max(1,(a|b).sum())),n))
        records.sort()
        sheet=Image.new("RGB",(960,6*264))
        with ZipFile(t.YOLODIR/"test_images.zip") as z:
            for i,(iou,n) in enumerate(records[:6]):
                im=cv2.imdecode(np.frombuffer(z.read(n),np.uint8),1)
                for j,arr in enumerate([None,before[n],after[n]]):
                    view=im.copy()
                    if arr is not None:
                        m=np.unpackbits(arr).reshape(im.shape[:2]).astype(bool)
                        view[m]=(view[m]*.57+[100,0,70]).clip(0,255).astype(np.uint8)
                    x,y=j*320,i*264;sheet.paste(Image.fromarray(view[:,:,::-1]).resize((320,240)),(x,y))
                    ImageDraw.Draw(sheet).text((x+3,y+240),n.split("_")[4]+"  "+["raw","expv4","expv4s"][j],fill="white")
        sheet.save(OUT/"largest_changes.jpg",quality=94)
    summary["other103"]=dict(mean_model_agreement=float(np.mean([x for x,_ in records])),
                             below85=sum(x<.85 for x,_ in records),worst=records[:6])
    (OUT/"independent_replay.json").write_text(json.dumps(summary,indent=2))
    print(summary["other103"])

if __name__=="__main__":
    main()
