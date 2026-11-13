"""Offline planar box-face feature tracking; no device or movement commands."""
import argparse
import hashlib
import json
from pathlib import Path
import uuid

import cv2
import numpy as np

def common_reference_area(corners,H,image_shape):
    """Visible face area in reference pixels under the measured homography."""
    rh,rw=image_shape
    inverse=np.linalg.inv(H)
    frame=np.array([[0,0],[rw-1,0],[rw-1,rh-1],[0,rh-1]],np.float32)
    homogeneous=np.c_[frame,np.ones(4)]@inverse.T
    depth=homogeneous[:,2]
    if np.any(abs(depth)<1e-8) or not (np.all(depth>0) or np.all(depth<0)):
        raise ValueError('projective horizon crosses image')
    back=(homogeneous[:,:2]/depth[:,None]).astype(np.float32)
    if not cv2.isContourConvex(back):raise ValueError('invalid current field of view')
    area,_=cv2.intersectConvexConvex(np.asarray(corners,np.float32),back)
    return float(area)


def rectified_face_points(reference,current,reference_quad,current_quad,ratio):
    """Match texture after plane/illumination normalization, then map actual
    feature locations back to original pixels. Never count inferred corners.
    """
    size=(420,136);target=np.float32([[0,0],[419,0],[419,135],[0,135]])
    A=cv2.getPerspectiveTransform(np.asarray(reference_quad,np.float32),target)
    B=cv2.getPerspectiveTransform(np.asarray(current_quad,np.float32),target)
    a=cv2.warpPerspective(reference,A,size);b=cv2.warpPerspective(current,B,size)
    ma=cv2.warpPerspective(np.full_like(reference,255),A,size)
    mb=cv2.warpPerspective(np.full_like(current,255),B,size)
    ma=cv2.erode(ma,np.ones((9,9),np.uint8));mb=cv2.erode(mb,np.ones((9,9),np.uint8))
    equalizer=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8))
    sift=cv2.SIFT_create(nfeatures=1500)
    k1,d1=sift.detectAndCompute(equalizer.apply(a),ma);k2,d2=sift.detectAndCompute(equalizer.apply(b),mb)
    if d1 is None or d2 is None or min(len(d1),len(d2))<12:return [],[]
    matcher=cv2.BFMatcher();pairs=matcher.knnMatch(d1,d2,k=2)
    reverse={m.queryIdx:m.trainIdx for m in matcher.match(d2,d1)}
    good=[m[0] for m in pairs if len(m)==2 and m[0].distance<ratio*m[1].distance and reverse.get(m[0].trainIdx)==m[0].queryIdx]
    if not good:return [],[]
    p=np.float32([k1[m.queryIdx].pt for m in good]);q=np.float32([k2[m.trainIdx].pt for m in good])
    return cv2.perspectiveTransform(p[None],np.linalg.inv(A))[0],cv2.perspectiveTransform(q[None],np.linalg.inv(B))[0]

def track(reference, current, corners, scale_bounds=(.4,2.5), ratio_threshold=.7,
          coverage_basis='full',current_face_hint=None):
    result = {"status":"needs_observation", "for_motion":False,
              "robot_target":None, "method":"masked_sift_planar_homography_prototype",
              "projected_corners_are_inferred":True,
              "physical_accuracy_verified":False}
    if reference.shape != current.shape or reference.ndim != 2:
        raise ValueError("需要相同尺寸的灰度原图")
    q = np.asarray(corners, np.float32)
    if q.shape != (4,2) or not np.isfinite(q).all() or not cv2.isContourConvex(q):
        raise ValueError("参考四角无效")
    area = abs(cv2.contourArea(q))
    if area < 100:
        raise ValueError("参考区域过小")
    mask = np.zeros_like(reference)
    cv2.fillConvexPoly(mask, np.rint(q).astype(np.int32), 255)
    mask = cv2.erode(mask, np.ones((9,9), np.uint8))
    sift = cv2.SIFT_create(nfeatures=1500)
    k1, d1 = sift.detectAndCompute(reference, mask)
    current_mask=None
    if current_face_hint is not None:
        hint=np.asarray(current_face_hint,np.float32)
        if hint.shape!=(4,2) or not np.isfinite(hint).all() or not cv2.isContourConvex(hint):
            return dict(result,reason='invalid_independent_current_face_hint')
        current_mask=np.zeros_like(current)
        cv2.fillConvexPoly(current_mask,np.rint(hint).astype(np.int32),255)
        margin=max(12,min(40,round(np.sqrt(abs(cv2.contourArea(hint)))*.1)))
        current_mask=cv2.dilate(current_mask,np.ones((margin*2+1,margin*2+1),np.uint8))
        result['matching_scope']='independent_pose_predicted_face_roi'
    k2, d2 = sift.detectAndCompute(current, current_mask)
    if d1 is None or d2 is None or len(d1)<12 or len(d2)<12:
        return dict(result, reason="insufficient_face_features")
    matcher = cv2.BFMatcher()
    pairs = matcher.knnMatch(d1,d2,k=2)
    reverse = matcher.match(d2,d1)
    reverse_map = {m.queryIdx:m.trainIdx for m in reverse}
    good = [v[0] for v in pairs if len(v)==2 and v[0].distance < ratio_threshold*v[1].distance
            and reverse_map.get(v[0].trainIdx)==v[0].queryIdx]
    # SIFT orientations at one pixel are not independent spatial observations.
    unique = {}
    for m in sorted(good,key=lambda v:v.distance):
        key = (tuple(np.round(k1[m.queryIdx].pt,1)), tuple(np.round(k2[m.trainIdx].pt,1)))
        unique.setdefault(key,m)
    good = list(unique.values())
    p1 = np.float32([k1[m.queryIdx].pt for m in good])
    p2 = np.float32([k2[m.trainIdx].pt for m in good])
    result['raw_match_count']=len(good)
    if current_face_hint is not None:
        a,b=rectified_face_points(reference,current,q,current_face_hint,ratio_threshold)
        result['rectified_match_count']=len(a)
        merged=[]
        for source,target_point in list(zip(p1,p2))+list(zip(a,b)):
            if any(np.linalg.norm(source-x)<1.0 or np.linalg.norm(target_point-y)<1.0 for x,y in merged):continue
            merged.append((source,target_point))
        p1=np.float32([x for x,y in merged]).reshape(-1,2);p2=np.float32([y for x,y in merged]).reshape(-1,2)
        result['matching_method']='raw_and_independently_rectified_texture_unique_original_pixel_points'
    result["match_count"] = len(p1)
    if len(p1)<12:return dict(result,reason="insufficient_mutual_matches")
    H, inliers = cv2.findHomography(p1,p2,cv2.RANSAC,2.5)
    if H is None or inliers is None or not np.isfinite(H).all():
        return dict(result,reason="homography_failed")
    keep = inliers.ravel().astype(bool)
    count = int(keep.sum())
    if count<12:
        return dict(result,reason="insufficient_inliers",inlier_count=count)
    projected = cv2.perspectiveTransform(p1[None],H)[0]
    median = float(np.median(np.linalg.norm(projected[keep]-p2[keep],axis=1)))
    coverage = float(cv2.contourArea(cv2.convexHull(p1[keep]))/area)
    rh,rw=reference.shape
    visible_area,_=cv2.intersectConvexConvex(q,np.array([[0,0],[rw-1,0],[rw-1,rh-1],[0,rh-1]],np.float32))
    visible_coverage=float(cv2.contourArea(cv2.convexHull(p1[keep]))/max(visible_area,1))
    # At a close view, part of a full overview reference is outside the
    # current sensor. Normalize by the genuinely shared field of view,
    # mapped back to reference pixels; don't demand features outside it.
    try:
        common_area=common_reference_area(q,H,reference.shape)
    except (np.linalg.LinAlgError,ValueError,cv2.error):
        return dict(result,reason='invalid_common_visible_region')
    common_coverage=float(cv2.contourArea(cv2.convexHull(p1[keep]))/max(common_area,1))
    common_fraction=float(common_area/area)
    effective_coverage=(common_coverage if coverage_basis=='common_visible' else
                        visible_coverage if coverage_basis=='visible_reference' else coverage)
    quad = cv2.perspectiveTransform(q[None],H)[0]
    scale = abs(cv2.contourArea(quad))/area
    result.update(inlier_count=count,inlier_ratio=count/len(p1),
                  median_inlier_error_px=median,reference_face_coverage=coverage,
                  reference_visible_face_coverage=visible_coverage,coverage_basis=coverage_basis,
                  common_visible_reference_fraction=common_fraction,
                  common_visible_reference_coverage=common_coverage)
    if (count/len(p1)<.6 or median>1.5 or effective_coverage<.2
            or (coverage_basis=='common_visible' and (common_area<100 or common_fraction<.15))
            or not np.isfinite(quad).all() or not cv2.isContourConvex(quad)
            or not scale_bounds[0]<scale<scale_bounds[1] or np.linalg.norm(np.roll(quad,-1,axis=0)-quad,axis=1).min()<12):
        return dict(result,reason="tracking_quality_rejected")
    center = q.mean(axis=0)
    tracked_center = cv2.perspectiveTransform(center[None,None],H)[0,0]
    height,width = current.shape
    visible = ((quad[:,0]>=0)&(quad[:,0]<width)&(quad[:,1]>=0)&(quad[:,1]<height))
    result.update(status="tracked_face_for_review",reason="distributed_planar_matches",
                  homography_reference_to_current=H.tolist(),
                  reference_center_pixel=center.tolist(),
                  tracked_reference_center_pixel=tracked_center.tolist(),
                  center_delta_pixel=(tracked_center-center).tolist(),
                  inferred_face_corners_pixel=quad.tolist(),
                  inferred_corners_inside_image=visible.tolist(),
                  inlier_source_pixel=p1[keep].tolist(),inlier_current_pixel=p2[keep].tolist())
    return result


def read_image(path,flag):
    frame = cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),flag)
    if frame is None:
        raise ValueError("不能读取原图")
    return frame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_outline",type=Path)
    parser.add_argument("image",type=Path)
    parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parent/"vision_grasp_results")
    args=parser.parse_args()
    outline=json.loads(args.reference_outline.read_text(encoding="utf-8"))
    if len(outline.get("candidates",[]))!=1:
        raise ValueError("参考轮廓必须只有一个已核对候选")
    reference=Path(outline["image_path"])
    if hashlib.sha256(reference.read_bytes()).hexdigest()!=outline["image_sha256"]:
        raise ValueError("参考图片校验不一致")
    result=track(read_image(reference,0),read_image(args.image,0),outline["candidates"][0]["corners_pixel"])
    result.update(reference_image=str(reference.resolve()),image=str(args.image.resolve()),
                  image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
                  reference_outline=str(args.reference_outline.resolve()),
                  limitations=["Planar same-face prototype; no occlusion or blur accuracy guarantee.",
                               "Inferred corners are not independently observed corners.",
                               "One motion pair does not calibrate a general robot/image Jacobian.",
                               "Tracking does not confirm grasp, height, identity or camera-to-robot calibration."])
    args.output.mkdir(parents=True,exist_ok=True)
    stem=args.image.stem+"_"+uuid.uuid4().hex[:8]+"_tracking"
    out=args.output/(stem+".json")
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    image=read_image(args.image,1)
    if result["status"]=="tracked_face_for_review":
        cv2.polylines(image,[np.rint(result["inferred_face_corners_pixel"]).astype(np.int32)],True,(0,220,255),2)
        for point in result["inlier_current_pixel"]:
            cv2.circle(image,tuple(np.rint(point).astype(int)),1,(0,220,0),-1)
        cv2.drawMarker(image,tuple(np.rint(result["tracked_reference_center_pixel"]).astype(int)),(0,0,255),cv2.MARKER_CROSS,14,2)
    cv2.putText(image,"INFERRED FACE TRACK - REVIEW ONLY",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,220,255),1)
    ok,png=cv2.imencode(".png",image)
    if not ok: raise ValueError("标注图保存失败")
    png.tofile(args.output/(stem+".png"))
    print(json.dumps({k:result.get(k) for k in ("status","reason","inlier_count","reference_face_coverage","median_inlier_error_px","center_delta_pixel")},ensure_ascii=False))
    print(out.resolve())


if __name__=="__main__":
    main()
