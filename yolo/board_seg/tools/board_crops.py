"""Make separately reported empty-board challenges from held-out views with labeled boxes."""
import csv

import cv2
import numpy as np
from prepare import DATA, WORK


def main():
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    rows = []
    for split in ("val", "test"):
        out = WORK/"challenge"/split
        out.mkdir(parents=True,exist_ok=True)
        for p in sorted((DATA/"images"/split).glob("photo_*.jpg")):
            image=cv2.imread(str(p))
            h,w=image.shape[:2]
            label=DATA/"labels"/split/(p.stem+".txt")
            if not label.read_text().strip():
                continue
            mask=np.zeros((h,w),np.uint8)
            for row in label.read_text().splitlines():
                q=(np.array(list(map(float,row.split()[1:]))).reshape(-1,2)*[w,h]).astype(np.int32)
                cv2.fillPoly(mask,[q],255)
            mask=cv2.dilate(mask,np.ones((21,21),np.uint8))
            candidates=[]
            for y in range(50,h-149,45):
                for x in range(0,w-179,45):
                    if mask[y:y+150,x:x+180].any():
                        continue
                    region=image[y:y+150,x:x+180]
                    _,ids,_=detector.detectMarkers(region)
                    if ids is not None and np.all(ids<17):
                        candidates.append((len(ids),x,y))
            if candidates:
                _,x,y=max(candidates)
                cv2.imwrite(str(out/(p.stem+".jpg")),image[y:y+150,x:x+180])
                rows.append((split,p.name,x,y,180,150))
    with (WORK/"challenge/crops.csv").open("w",newline="") as f:
        writer=csv.writer(f)
        writer.writerow(("split","source","x","y","width","height"))
        writer.writerows(rows)
    print("Marker-positive, box-free crops:",len(rows))


if __name__=="__main__":
    main()
