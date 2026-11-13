"""Register a newly placed board from a stationary high observation, no writes to hardware.

Uses the existing camera mount and nominal arm pose to anchor today's board.
This retains nominal robot uncertainty; subsequent proximity uses visual feedback.
"""
import argparse,copy,json
from pathlib import Path
import numpy as np
import cv2
from check_grasp_board import ROOT,inspect
from fit_grasp_handeye import camera_candidates
from observe_grasp_target import locate
from board_marker_pose import marker_plane_candidates_with_contrast
from grasp_motion_profile import DEFAULT_FIXED_WRIST_ALIGNMENT_DEG,load_motion_profile

def robust_board_pose_candidates(board,K,D):
    """Use the RANSAC-supported corner set consistently in metric PnP.

    The visibility stage permits <=10% rejected interpolation points; those
    points must not be fed back into the pose solve. Keep raw evidence and
    rejected IDs so a successful inlier fit never hides them.
    """
    ids=np.array(board['corner_ids']).reshape(-1)
    img=np.array(board['corners_px'],np.float64)
    size=board['profile']['square_length'];columns=board['profile']['squares_x']-1
    obj=np.array([[(int(i)%columns+1)*size,(int(i)//columns+1)*size,0] for i in ids],np.float64)
    undistorted=cv2.undistortPoints(img.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    H,mask=cv2.findHomography(obj[:,:2],undistorted,cv2.RANSAC,1.5)
    if H is None or mask is None:raise ValueError('板角点稳健拟合失败')
    keep=mask.ravel().astype(bool)
    if keep.sum()<12 or keep.mean()<.9:raise ValueError('板位姿有效角点比例不足')
    filtered={**board,'corner_ids':ids[keep].tolist(),'corners_px':img[keep].tolist()}
    solutions=[]
    for C,_ in camera_candidates(filtered,K,D):
        rv,tv=cv2.solvePnPRefineLM(obj[keep],img[keep],K,D,cv2.Rodrigues(C[:3,:3])[0],C[:3,3].reshape(3,1))
        R=cv2.Rodrigues(rv)[0];pred=cv2.projectPoints(obj[keep],rv,tv,K,D)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((pred-img[keep])**2,axis=1))))
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=tv.ravel()
        if np.min((R@obj[keep].T+tv)[2])<=0:continue
        if any(np.linalg.norm(T[:3,3]-p[:3,3])<.1 and np.linalg.norm(cv2.Rodrigues(T[:3,:3]@p[:3,:3].T)[0])<np.radians(.5) for p,e in solutions):continue
        solutions.append((T,error))
    metadata={'raw_corner_count':len(ids),'pose_inlier_count':int(keep.sum()),
              'rejected_corner_ids':ids[~keep].tolist(),'inlier_ratio':float(keep.mean()),
              'ransac_threshold_px':1.5,'pose_rmse_limit_px':.6,'raw_evidence_retained':True}
    return sorted(solutions,key=lambda row:row[1]),metadata

def register(path,output_dir=None,*,motion=None):
    o=json.loads(path.read_text(encoding='utf-8'))
    b=inspect(Path(o['image']),ROOT/'calibration_boards/charuco_7x5_print96_v1.json')
    k=json.loads((ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json').read_text(encoding='utf-8'))
    K,D=np.array(k['camera_matrix_px']),np.array(k['distortion_coefficients'])
    image=cv2.imdecode(np.frombuffer(Path(o['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),1.44,.96,dictionary)
    parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    solutions,pose_validation=marker_plane_candidates_with_contrast(
        cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),K,D,board,cv2.aruco.ArucoDetector(dictionary,parameters),
        allow_isolated_outlier=True)
    method='decoded_marker_centers'
    if (not solutions or (len(solutions)>1 and solutions[1][1]-solutions[0][1]<.5)) and b['reference_candidate']:
        try:
            alternate,corner_validation=robust_board_pose_candidates(b,K,D)
            if alternate and alternate[0][1]<=.6 and (len(alternate)==1 or alternate[1][1]-alternate[0][1]>=.5):
                solutions,pose_validation=alternate,{'method':'robust_charuco_fallback','marker_diagnostics':pose_validation,'corner_diagnostics':corner_validation}
                method='robust_charuco_fallback'
        except ValueError as error:pose_validation['charuco_fallback_rejection']=str(error)
    if not solutions:raise ValueError('真实编码标记和角点均未提供可靠板位姿：'+json.dumps(pose_validation,ensure_ascii=False))
    if solutions[0][1]>.6:raise ValueError('本帧板位姿残差不合格')
    if len(solutions)>1 and solutions[1][1]-solutions[0][1]<.5:raise ValueError('本帧板位姿仍存在歧义')
    cal=copy.deepcopy(json.loads(Path(o['calibration']).read_text(encoding='utf-8')))
    best=next(c for c in cal['candidates'] if c['mount_hypothesis']=='pitch_tool')
    camera=np.array(o['geometry']['T_base_camera_cm'])
    best['T_base_board_cm']=(camera@solutions[0][0]).tolist()
    cal['scene_registration']={'source':str(path.resolve()),'basis':'decoded_current_board_and_nominal_robot_camera_mount','measurement_method':method,'board_pnp_rmse_px':solutions[0][1],
        'board_pose_validation':pose_validation,'physical_accuracy_verified':False}
    directory=path.parent if output_dir is None else Path(output_dir)
    directory.mkdir(parents=True,exist_ok=True)
    calpath=directory/(path.stem+'_scene_calibration.json')
    calpath.write_text(json.dumps(cal,ensure_ascii=False,indent=2),encoding='utf-8')
    image=cv2.imdecode(np.frombuffer(Path(o['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    o['calibration']=str(calpath.resolve())
    o['geometry']=locate(image,o['ocr'],o['before']['joint_median'],best,camera_override=camera)
    o['scene_registration']=cal['scene_registration']
    if o['geometry']['selected'] is None:
        outline=o['geometry']['outline']
        fit=outline.get('metric_edge_fit')
        detail=outline['reason']
        if fit:
            reason='拟合得到的盒顶角点不一致，' if fit.get('rejection_reason')=='edge_fits_disagree_on_box_corners' else ''
            detail=(f"{reason}边缘中位误差 {fit['edge_median_px']:.2f}px，"
                    f"P90 {fit['edge_p90_px']:.2f}px，蓝色覆盖 {fit['blue_fraction_inside']:.1%}")
        raise ValueError('新场景中药盒边界拟合失败，'+detail)
    # Wrist-roll alignment has not been calibrated for arbitrary orientations.
    # Reject unsupported yaw before any movement, rather than claiming that
    # planar recognition alone makes all box orientations graspable.
    target=o['geometry']['selected']
    center=np.array(target['top_center_base_cm'])
    down=np.array(o['geometry']['table_down_normal_base'])
    o['scene_registration']['nominal_scene_geometry']={
        'box_top_center_base_cm':center.tolist(),
        'support_at_box_center_base_cm':(center+2.5*down).tolist(),
        'support_down_normal_base':down.tolist(),
    }
    corners=np.array(target['corners_pixel'])
    if np.any(corners[:,0]<2) or np.any(corners[:,0]>image.shape[1]-2) or np.any(corners[:,1]<2) or np.any(corners[:,1]>image.shape[0]-2):
        raise ValueError('初始药盒未完整入镜；本版尚不自动搜索视野')
    axis=np.array(target['length_axis_base'])[:2]
    radial=np.array(target['top_center_base_cm'])[:2]
    alignment=undirected_angle_deg(axis,radial)
    limit=(motion or {}).get('maximum_fixed_wrist_alignment_deg',DEFAULT_FIXED_WRIST_ALIGNMENT_DEG)
    o['grasp_scope']={'profile':'same_box_radial_trial_v1','long_axis_to_radial_deg':alignment,
        'maximum_supported_orientation_assumption_deg':limit,'arbitrary_wrist_alignment_supported':False}
    if alignment>limit:
        raise ValueError(f'药盒长轴相对底座径向偏 {alignment:.1f}°，超过固定腕轴试验上限 {limit:g}°')
    out=directory/(path.stem+'_registered.json')
    out.write_text(json.dumps(o,ensure_ascii=False,indent=2),encoding='utf-8')
    print('新场景注册：',out,'板误差px：',solutions[0][1],'目标中心cm：',o['geometry']['selected']['top_center_base_cm'])
    return out

def undirected_angle_deg(axis,reference):
    scale=np.linalg.norm(axis)*np.linalg.norm(reference)
    if not np.isfinite(scale) or scale<=0:raise ValueError('无法计算对齐角度')
    return float(np.degrees(np.arccos(np.clip(abs(axis@reference)/scale,0,1))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('observation',type=Path)
    p.add_argument('--motion-profile',type=Path)
    a=p.parse_args()
    register(a.observation,motion=load_motion_profile(a.motion_profile))
