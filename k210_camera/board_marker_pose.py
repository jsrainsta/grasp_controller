"""Shared decoded-marker board measurement for registration and close views.

No inferred ChArUco intersections are required. Uses decoded marker centers,
spatial support, full marker-outline cross-checks, and positive-depth PnP.
"""
import cv2
import numpy as np

def marker_plane_candidates_with_contrast(gray,K,D,board,detector,*,allow_three=False,
                                          allow_isolated_outlier=False,allow_wide_inlier_support=False):
    poses,metadata=marker_plane_candidates(gray,K,D,board,detector,allow_three=allow_three,
                                          allow_isolated_outlier=allow_isolated_outlier,
                                          allow_wide_inlier_support=allow_wide_inlier_support)
    subset_accepted=metadata.get('layout_acceptance')=='large_board_cross_checked_subset'
    if poses and not subset_accepted:
        return poses,metadata
    enhanced=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(gray)
    retry,retry_meta=marker_plane_candidates(enhanced,K,D,board,detector,allow_three=allow_three,
                                            allow_isolated_outlier=allow_isolated_outlier,
                                            allow_wide_inlier_support=allow_wide_inlier_support)
    if retry and not (poses and retry_meta.get('layout_acceptance')=='large_board_cross_checked_subset'):
        retry_meta['image_preprocessing']='clahe'
        retry_meta['raw_marker_rejection']=metadata.get('rejection')
        return retry,retry_meta
    if poses or not allow_wide_inlier_support:
        return poses,metadata
    remeasured,remeasured_meta=marker_plane_candidates(
        gray,K,D,board,EnlargedContourDetector(board),allow_three=allow_three,
        allow_isolated_outlier=allow_isolated_outlier,allow_wide_inlier_support=True,
        allow_corner_layout=allow_three)
    if remeasured:
        remeasured_meta['image_preprocessing']='2x_contour_corner_refinement'
        return remeasured,remeasured_meta
    metadata['remeasured_board_rejection']=remeasured_meta
    return poses,metadata


class EnlargedContourDetector:
    def __init__(self,board):
        parameters=cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR
        self.detector=cv2.aruco.ArucoDetector(board.getDictionary(),parameters)

    def detectMarkers(self,gray):
        enlarged=cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
        corners,ids,rejected=self.detector.detectMarkers(enlarged)
        return tuple((marker+.5)/2-.5 for marker in corners),ids,rejected


def held_out_marker_errors(obj, pixels, corners, observed_corners, K, D, rv, tv, *, use_corners=False):
    """Refit without each marker and measure that marker independently."""
    center_errors, corner_errors = [], []
    for index in range(len(obj)):
        keep = np.arange(len(obj)) != index
        if use_corners:
            training_obj=corners[keep].reshape(-1,3)
            training_pixels=observed_corners[keep].reshape(-1,2)
        else:
            training_obj,training_pixels=obj[keep],pixels[keep]
        if cv2.contourArea(cv2.convexHull(training_obj[:, :2].astype(np.float32))) < 2:
            return None
        rotation, translation = cv2.solvePnPRefineLM(
            training_obj, training_pixels, K, D, rv.copy(), tv.copy())
        if use_corners:
            R=cv2.Rodrigues(rotation)[0]
            relative_rotation=cv2.Rodrigues(R@cv2.Rodrigues(rv)[0].T)[0]
            if (np.linalg.norm(relative_rotation)>np.radians(2)
                    or np.linalg.norm(translation-tv)>.2
                    or np.min((corners.reshape(-1,3)@R.T+translation.ravel())[:,2])<=0):
                return None
        prediction = cv2.projectPoints(corners[index], rotation, translation, K, D)[0].reshape(-1, 2)
        center = cv2.projectPoints(obj[index:index+1], rotation, translation, K, D)[0].reshape(2)
        center_errors.append(float(np.linalg.norm(center - pixels[index])))
        corner_errors.append(float(np.max(np.linalg.norm(prediction - observed_corners[index], axis=1))))
    return max(center_errors), max(corner_errors)


def marker_plane_candidates(gray,K,D,board,detector,*,allow_three=False,
                            allow_isolated_outlier=False,allow_wide_inlier_support=False,
                            allow_corner_layout=False):
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
    corner_layout=False
    if three:
        valid=np.ones(3,dtype=bool)
    else:
        H,mask=cv2.findHomography(obj[:,:2],undistorted,cv2.RANSAC,1.5)
        if H is None or mask is None:
            # The centers can be almost collinear even though the actual
            # marker corners span two dimensions. Require support across rows.
            center_hull=cv2.convexHull(obj[:,:2].astype(np.float32))
            corner_layout=(allow_corner_layout and len(set(kept_ids))>=5
                           and len(set(kept_ids))==len(pairs) and cv2.contourArea(center_hull)>=2)
            if not corner_layout:
                return [],{**meta,'rejection':'marker_layout_fit_failed'}
            valid=np.ones(len(pairs),dtype=bool)
        else:
            valid=mask.ravel().astype(bool)
    area=float(cv2.contourArea(cv2.convexHull(obj[valid,:2].astype(np.float32))))
    count=int(valid.sum())
    ratio=float(valid.mean())
    meta.update(layout_inlier_count=count,layout_inlier_ratio=ratio,layout_hull_area_cm2=area,
                rejected_marker_ids=[j for j,ok in zip(kept_ids,valid) if not ok])
    isolated_outlier=allow_isolated_outlier and count>=5 and count==len(pairs)-1 and area>=20
    # A large, well-distributed set can tolerate a few mislocalized detections.
    # All retained markers still face the same metric pose and outline checks.
    wide_support=allow_wide_inlier_support and count>=12 and ratio>=.85 and area>=40
    layout_rejected=ratio<.9 and not (isolated_outlier or wide_support)
    # Preserve existing passes. On the large board, allow a strong majority
    # to proceed to independent marker checks even if extra detections disagree.
    cross_check_subset=allow_wide_inlier_support and layout_rejected and (
        (count>=12 and area>=40 and count>len(pairs)/2)
        or (allow_three and count>=5 and count==len(pairs)-1 and area>=2))
    if count<(3 if three else 4) or area<2 or (layout_rejected and not cross_check_subset):
        return [],{**meta,'rejection':'insufficient_spatial_marker_support'}
    if ratio<.9:
        if cross_check_subset:
            meta['layout_acceptance']='large_board_subset_requires_holdout'
        else:
            meta['layout_acceptance']=('one_outlier_with_wide_inlier_support' if isolated_outlier
                                       else 'large_board_wide_inlier_support')
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
    fit_obj,fit_pixels=(allobj,allpixels) if (three or corner_layout) else (obj,pixels)
    if corner_layout:
        meta['pose_support']='decoded_marker_corners_cross_checked'
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
        if cross_check_subset or corner_layout:
            held_out=held_out_marker_errors(obj,pixels,allobj.reshape(-1,4,3),
                                            allpixels.reshape(-1,4,2),K,D,rv,tv,use_corners=corner_layout)
            if held_out is None:
                diagnostics[-1]['subset_holdout_rejection']='insufficient_remaining_spatial_support'
                continue
            center_max,corner_max=held_out
            diagnostics[-1].update(held_out_center_max_px=center_max,held_out_corner_max_px=corner_max)
            if center_max>1.5 or corner_max>3.5:
                diagnostics[-1]['subset_holdout_rejection']='held_out_marker_error'
                continue
        C=np.eye(4);C[:3,:3]=R;C[:3,3]=tv.ravel()
        if any(np.linalg.norm(C[:3,3]-p[:3,3])<.1 and np.linalg.norm(cv2.Rodrigues(C[:3,:3]@p[:3,:3].T)[0])<np.radians(.5) for p,e in result):continue
        result.append((C,error))
    meta['candidate_errors']=diagnostics
    if cross_check_subset:
        if result:
            meta['layout_acceptance']='large_board_cross_checked_subset'
        else:
            meta['rejection']='large_board_subset_pose_not_verified'
    if corner_layout and not result:
        meta['rejection']='marker_corner_layout_not_verified'
    if result:
        meta.update(decoded_pose_accepted=True,measured_points_board_cm=obj.tolist(),
                    measured_points_pixel=pixels.tolist(),measurement_weight=16,
                    weight_basis='decoded center versus object texture uncertainty; separate residual gates retained')
    return sorted(result,key=lambda p:p[1]),meta
