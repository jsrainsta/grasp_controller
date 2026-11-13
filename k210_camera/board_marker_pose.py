"""Shared decoded-marker board measurement for registration and close views.

No inferred ChArUco intersections are required. Uses decoded marker centers,
spatial support, full marker-outline cross-checks, and positive-depth PnP.
"""
import cv2
import numpy as np

def marker_plane_candidates_with_contrast(gray,K,D,board,detector,*,allow_three=False,allow_isolated_outlier=False):
    poses,metadata=marker_plane_candidates(gray,K,D,board,detector,allow_three=allow_three,
                                          allow_isolated_outlier=allow_isolated_outlier)
    if poses:
        return poses,metadata
    enhanced=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(gray)
    retry,retry_meta=marker_plane_candidates(enhanced,K,D,board,detector,allow_three=allow_three,
                                            allow_isolated_outlier=allow_isolated_outlier)
    if not retry:
        return poses,metadata
    retry_meta['image_preprocessing']='clahe'
    retry_meta['raw_marker_rejection']=metadata.get('rejection')
    return retry,retry_meta

def marker_plane_candidates(gray,K,D,board,detector,*,allow_three=False,allow_isolated_outlier=False):
    corners,ids,_=detector.detectMarkers(gray)
    lookup={int(j):i for i,j in enumerate(board.getIds())}
    pairs=[];kept_ids=[]
    if ids is not None:
        for c,j in zip(corners,ids.ravel()):
            if int(j) in lookup:
                pairs.append((board.getObjPoints()[lookup[int(j)]],c.reshape(4,2)));kept_ids.append(int(j))
    meta={'marker_ids':kept_ids,'marker_count':len(pairs),'pose_support':'decoded_marker_centers_with_outer_corner_check'}
    three=allow_three and len(pairs)==3 and len(set(kept_ids))==3
    if len(pairs)<4 and not three:return [],{**meta,'rejection':'insufficient_decoded_markers'}
    obj=np.array([p[0].mean(0) for p in pairs],np.float64);pixels=[]
    for _,q in pairs:
        a=np.cross(np.r_[q[0],1],np.r_[q[2],1]);b=np.cross(np.r_[q[1],1],np.r_[q[3],1]);c=np.cross(a,b)
        if abs(c[2])<1e-8:return [],{**meta,'rejection':'invalid_marker_diagonals'}
        pixels.append(c[:2]/c[2])
    pixels=np.array(pixels,np.float64)
    undistorted=cv2.undistortPoints(pixels.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    if three:
        valid=np.ones(3,dtype=bool)
    else:
        H,mask=cv2.findHomography(obj[:,:2],undistorted,cv2.RANSAC,1.5)
        if H is None or mask is None:return [],{**meta,'rejection':'marker_layout_fit_failed'}
        valid=mask.ravel().astype(bool)
    area=float(cv2.contourArea(cv2.convexHull(obj[valid,:2].astype(np.float32))))
    meta.update(layout_inlier_count=int(valid.sum()),layout_inlier_ratio=float(valid.mean()),layout_hull_area_cm2=area,
                rejected_marker_ids=[j for j,ok in zip(kept_ids,valid) if not ok])
    isolated_outlier=(allow_isolated_outlier and not three and valid.sum()>=5
                      and valid.sum()==len(pairs)-1 and area>=20)
    if valid.sum()<(3 if three else 4) or (valid.mean()<.9 and not isolated_outlier) or area<2:
        return [],{**meta,'rejection':'insufficient_spatial_marker_support'}
    if isolated_outlier and valid.mean()<.9:
        meta['layout_acceptance']='one_outlier_with_wide_inlier_support'
    obj,pixels=obj[valid],pixels[valid]
    allobj=np.concatenate([p[0] for p,ok in zip(pairs,valid) if ok]).astype(np.float64)
    allpixels=np.concatenate([p[1] for p,ok in zip(pairs,valid) if ok]).astype(np.float64)
    if three:
        cross_errors=[]
        for excluded in range(3):
            keep=np.repeat(np.arange(3)!=excluded,4)
            fitted=cv2.solvePnPGeneric(allobj[keep],allpixels[keep],K,D,flags=cv2.SOLVEPNP_IPPE)
            errors=[]
            for rv,tv in zip(fitted[1],fitted[2]):
                rv,tv=cv2.solvePnPRefineLM(allobj[keep],allpixels[keep],K,D,rv,tv)
                if np.min((cv2.Rodrigues(rv)[0]@allobj.T+tv)[2])<=0:continue
                projection=cv2.projectPoints(allobj,rv,tv,K,D)[0].reshape(-1,2)
                errors.append(float(np.max(np.linalg.norm(projection-allpixels,axis=1))))
            cross_errors.append(min(errors,default=float('inf')))
        meta['three_marker_cross_check_max_px']=[e if np.isfinite(e) else None for e in cross_errors]
        if max(cross_errors)>3.5:return [],{**meta,'rejection':'three_marker_cross_check_failed'}
    fit_obj,fit_pixels=(allobj,allpixels) if three else (obj,pixels)
    if three:
        meta['pose_support']='three_decoded_markers_all_corners_checked'
    solved=cv2.solvePnPGeneric(fit_obj,fit_pixels,K,D,flags=cv2.SOLVEPNP_IPPE);result=[]
    # Four centers can include three collinear points. The measured outer
    # corners still provide a nondegenerate planar pose initialization.
    if not solved[0]:
        solved=cv2.solvePnPGeneric(allobj,allpixels,K,D,flags=cv2.SOLVEPNP_IPPE)
        meta['pose_initialization']='decoded_marker_outer_corners'
    if not solved[0]:return [],{**meta,'rejection':'marker_pnp_failed'}
    diagnostics=[]
    for rv,tv in zip(solved[1],solved[2]):
        rv,tv=cv2.solvePnPRefineLM(fit_obj,fit_pixels,K,D,rv,tv)
        R=cv2.Rodrigues(rv)[0]
        if np.min((R@allobj.T+tv)[2])<=0:continue
        projection=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((projection-pixels)**2,axis=1))))
        outer=cv2.projectPoints(allobj,rv,tv,K,D)[0].reshape(-1,2)
        outer_error=float(np.max(np.linalg.norm(outer-allpixels,axis=1)))
        diagnostics.append({'center_rmse_px':error,'outer_corner_max_px':outer_error})
        if error>.6 or outer_error>3.5:continue
        C=np.eye(4);C[:3,:3]=R;C[:3,3]=tv.ravel()
        if any(np.linalg.norm(C[:3,3]-p[:3,3])<.1 and np.linalg.norm(cv2.Rodrigues(C[:3,:3]@p[:3,:3].T)[0])<np.radians(.5) for p,e in result):continue
        result.append((C,error))
    meta['candidate_errors']=diagnostics
    if result:
        meta.update(decoded_pose_accepted=True,measured_points_board_cm=obj.tolist(),
                    measured_points_pixel=pixels.tolist(),measurement_weight=16,
                    weight_basis='decoded center versus object texture uncertainty; separate residual gates retained')
    return sorted(result,key=lambda p:p[1]),meta
