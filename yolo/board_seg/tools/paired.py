"""Reconstruct empty backgrounds using only unoccluded pixels from registered training photographs."""
import warnings
import cv2
import numpy as np
from prepare import DATA, WORK, write_item


def load(p, detector):
    im=cv2.imread(str(p))
    corners,ids,_=detector.detectMarkers(im)
    markers={} if ids is None else {int(i):q.reshape(4,2) for i,q in zip(ids.ravel(),corners) if i<17}
    mask=np.zeros(im.shape[:2],np.uint8)
    for row in (DATA/"labels/train"/(p.stem+".txt")).read_text().splitlines():
        xy=np.array(list(map(float,row.split()[1:]))).reshape(-1,2)
        cv2.fillPoly(mask,[(xy*[im.shape[1],im.shape[0]]).astype(np.int32)],255)
    mask=cv2.dilate(mask,np.ones((19,19),np.uint8))
    return p,im,markers,mask


def main():
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    sources=[load(p,detector) for p in sorted((DATA/"images/train").glob("photo*"))]
    files=[]
    for path,im,markers,mask in sources:
        donors=[]
        for other,src,ids,occupied in sources:
            keys=sorted(markers.keys() & ids.keys())
            if path==other or len(keys)<3:
                continue
            points=np.concatenate([ids[k] for k in keys])
            dest=np.concatenate([markers[k] for k in keys])
            H,inliers=cv2.findHomography(points,dest,cv2.RANSAC,2.0)
            if H is None or inliers.mean()<.9:
                continue
            h,w=mask.shape
            free=cv2.warpPerspective(255-occupied,H,(w,h),flags=cv2.INTER_NEAREST)
            donor=cv2.warpPerspective(src,H,(w,h)).astype(np.float32)
            donor[free==0]=np.nan
            donors.append(donor)
        if not donors:
            continue
        with warnings.catch_warnings():
            warnings.simplefilter("ignore",category=RuntimeWarning)
            fill=np.nanmedian(np.stack(donors),axis=0)
        holes=(~np.isfinite(fill[:,:,0])) & (mask>0)
        if holes.sum() > .005*(mask>0).sum():
            continue
        out=im.copy()
        out[mask>0]=np.nan_to_num(fill[mask>0]).astype(np.uint8)
        if holes.any():
            out=cv2.inpaint(out,holes.astype(np.uint8),3,cv2.INPAINT_TELEA)
        files.append(write_item("paired","empty_"+path.stem,out,[]))
    # Final adaptation emphasizes native images while retaining part of the external mix.
    public=[p for p in (DATA/"mixed.txt").read_text().splitlines() if "/public/" in p.replace("\\","/")]
    base=(DATA/"train.txt").read_text(encoding="utf8")
    (DATA/"paired.txt").write_text(base+"".join(str(p)+"\n" for p in public[::3]+files),encoding="utf8")
    (DATA/"paired.yaml").write_text((DATA/"mixed.yaml").read_text().replace("train: mixed.txt","train: paired.txt"))
    print("Reconstructed empty backgrounds:",len(files))


if __name__=="__main__":
    main()
