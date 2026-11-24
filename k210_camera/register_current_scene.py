"""Register a newly placed board from a stationary high observation, no writes to hardware.

Uses the existing camera mount and nominal arm pose to anchor today's board.
This retains nominal robot uncertainty; subsequent proximity uses visual feedback.
"""
import argparse,copy,json
from pathlib import Path
import numpy as np
import cv2
from check_grasp_board import LARGE_GRASP_BOARD_PROFILE,ROOT,inspect,make_board
from fit_grasp_handeye import camera_candidates, discrepancy
from observe_grasp_target import clipped_box_top,locate
from box_rim_geometry import RimEvidence,face_corners,refine_grounded_rim
from board_marker_pose import marker_plane_candidates_with_contrast
from grasp_motion_profile import DEFAULT_FIXED_WRIST_ALIGNMENT_DEG,load_motion_profile

def robust_board_pose_candidates(board,K,D,*,minimum_hull_fraction=None):
    """Use the RANSAC-supported corner set consistently in metric PnP.

    The visibility stage permits <=10% rejected interpolation points; those
    points must not be fed back into the pose solve. Keep raw evidence and
    rejected IDs so a successful inlier fit never hides them.
    """
    ids=np.array(board['corner_ids']).reshape(-1)
    img=np.array(board['corners_px'],np.float64)
    profile=board['profile']
    size=profile['square_length'];columns=profile['squares_x']-1
    obj=np.array([[(int(i)%columns+1)*size,(int(i)//columns+1)*size,0] for i in ids],np.float64)
    undistorted=cv2.undistortPoints(img.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    H,mask=cv2.findHomography(obj[:,:2],undistorted,cv2.RANSAC,1.5)
    if H is None or mask is None:raise ValueError('板角点稳健拟合失败')
    keep=mask.ravel().astype(bool)
    if keep.sum()<12 or keep.mean()<.9:raise ValueError('板位姿有效角点比例不足')
    coverage={}
    if minimum_hull_fraction is not None:
        hull=cv2.convexHull(obj[keep,:2].astype(np.float32))
        full_area=(columns-1)*(profile['squares_y']-2)*size**2
        fraction=float(cv2.contourArea(hull))/full_area
        if fraction<minimum_hull_fraction:
            raise ValueError(f'板位姿有效角点覆盖不足：{fraction:.1%}，要求 {minimum_hull_fraction:.0%}')
        coverage={'pose_inlier_hull_fraction':fraction}
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
              'ransac_threshold_px':1.5,'pose_rmse_limit_px':.6,'raw_evidence_retained':True,**coverage}
    return sorted(solutions,key=lambda row:row[1]),metadata

class BoardPoseAmbiguous(ValueError):
    pass


class ObservationBoxEdgeUnclear(ValueError):
    def __init__(self,message,board_pose,upper_edge_pixels=None):
        super().__init__(message)
        self.board_pose=board_pose
        self.upper_edge_pixels=upper_edge_pixels


def verify_stationary_box(previous, current_image, board_pose, camera, K, D):
    old_cal=json.loads(Path(previous['calibration']).read_text(encoding='utf-8'))
    old_board=np.asarray(next(c for c in old_cal['candidates']
                              if c['mount_hypothesis']=='pitch_tool')['T_base_board_cm'])
    translation, rotation=discrepancy(old_board,board_pose)
    if translation>.5 or rotation>3:
        raise ValueError(f'抬高后标定板位置不一致（{translation:.2f} cm，{rotation:.2f}°）')
    geometry=previous['geometry']
    target=geometry['selected']
    axis=np.asarray(target['length_axis_base'])
    normal=old_board[:3,2]
    box=np.eye(4)
    box[:3,:3]=np.column_stack((axis,np.cross(normal,axis),normal))
    box[:3,3]=target['top_center_base_cm']
    current_camera=old_board@np.linalg.solve(board_pose,camera)
    nominal=np.linalg.solve(current_camera,box)
    evidence=RimEvidence(current_image,K,D,face_corners(geometry['assumed_box_dimensions_cm']))
    fit=refine_grounded_rim(evidence,nominal,allow_cropped=True)
    if fit is None:
        raise ValueError('抬高后可见盒边不足，无法确认药盒仍在原位')
    pose,rim=fit
    shift,yaw=discrepancy(nominal,pose)
    if shift>.3 or yaw>3:
        raise ValueError(f'抬高后药盒轮廓位置改变（{shift:.2f} cm，{yaw:.2f}°）')
    return {'board_shift_cm':translation,'board_rotation_deg':rotation,
            'box_shift_cm':shift,'box_rotation_deg':yaw,'box_edge_coverage':rim['coverage']}


def register(path,output_dir=None,*,motion=None,saved_plane=False,previous_registration=None,
             board_profile=None):
    o=json.loads(path.read_text(encoding='utf-8'))
    previous=None
    if saved_plane:
        cal=copy.deepcopy(json.loads(Path(o['calibration']).read_text(encoding='utf-8')))
        if cal.get('scene_registration',{}).get('basis')!='decoded_current_board_and_nominal_robot_camera_mount':
            raise ValueError('无板预览需要带板注册生成的台面文件')
        best=next(c for c in cal['candidates'] if c['mount_hypothesis']=='pitch_tool')
        image=cv2.imdecode(np.frombuffer(Path(o['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        camera=np.array(o['geometry']['T_base_camera_cm'])
        cal['scene_registration']={**cal['scene_registration'],'basis':'saved_support_plane_preview',
                                   'saved_calibration':o['calibration'],'board_pnp_rmse_px':None}
    else:
        b=inspect(Path(o['image']),board_profile)
        if previous_registration is not None:
            previous=json.loads(Path(previous_registration).read_text(encoding='utf-8'))
            previous_profile=previous['scene_registration'].get('board_profile')
            if previous_profile is not None and previous_profile != b['profile']:
                raise ValueError('同一次抓取的标定板发生变化，请从观察位重新开始')
        k=json.loads((ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json').read_text(encoding='utf-8'))
        K,D=np.array(k['camera_matrix_px']),np.array(k['distortion_coefficients'])
        image=cv2.imdecode(np.frombuffer(Path(o['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        board=make_board(b['profile'])
        large_board=b['profile'].get('board_id')==LARGE_GRASP_BOARD_PROFILE.stem
        parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        solutions,pose_validation=marker_plane_candidates_with_contrast(
            cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),K,D,board,cv2.aruco.ArucoDetector(board.getDictionary(),parameters),
            allow_isolated_outlier=True,allow_wide_inlier_support=large_board)
        method='decoded_marker_centers'
        # The visibility check fits distorted pixels. For the large board let
        # the pose solver assess coverage on its own undistorted inlier set.
        fallback_available=b['reference_candidate'] or (large_board and b['corner_count']>=12)
        if (not solutions or (len(solutions)>1 and solutions[1][1]-solutions[0][1]<.5)) and fallback_available:
            try:
                alternate,corner_validation=robust_board_pose_candidates(
                    b,K,D,minimum_hull_fraction=1/3 if large_board else None)
                if alternate and alternate[0][1]<=.6 and (len(alternate)==1 or alternate[1][1]-alternate[0][1]>=.5):
                    solutions,pose_validation=alternate,{'method':'robust_charuco_fallback','marker_diagnostics':pose_validation,'corner_diagnostics':corner_validation}
                    method='robust_charuco_fallback'
            except ValueError as error:pose_validation['charuco_fallback_rejection']=str(error)
        if not solutions:raise ValueError('真实编码标记和角点均未提供可靠板位姿：'+json.dumps(pose_validation,ensure_ascii=False))
        if solutions[0][1]>.6:raise ValueError('本帧板位姿残差不合格')
        if len(solutions)>1 and solutions[1][1]-solutions[0][1]<.5:raise BoardPoseAmbiguous('本帧板位姿仍存在歧义')
        cal=copy.deepcopy(json.loads(Path(o['calibration']).read_text(encoding='utf-8')))
        best=next(c for c in cal['candidates'] if c['mount_hypothesis']=='pitch_tool')
        camera=np.array(o['geometry']['T_base_camera_cm'])
        best['T_base_board_cm']=(camera@solutions[0][0]).tolist()
        cal['scene_registration']={'source':str(path.resolve()),'basis':'decoded_current_board_and_nominal_robot_camera_mount','measurement_method':method,'board_pnp_rmse_px':solutions[0][1],
            'board_pose_validation':pose_validation,'board_profile':b['profile'],'physical_accuracy_verified':False}
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
            detail=f"可见边覆盖 {fit['edge_coverage']}，边缘平均偏差 {fit['mean_edge_distance_px']:.2f}px"
        message='新场景中药盒边界拟合失败，'+detail
        if not saved_plane:
            frame_points=clipped_box_top(image,o['ocr'])
            if frame_points is not None:
                raise ObservationBoxEdgeUnclear(
                    f'{message}；已确认药名对应盒面接触画面顶部，先抬头补拍',
                    best['T_base_board_cm'],frame_points)
        if previous is not None:
            try:
                evidence=verify_stationary_box(previous,cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),
                                               np.asarray(best['T_base_board_cm']),camera,K,D)
            except ValueError as error:
                raise ValueError(f'{message}。{error}') from error
            # Preserve the proven image and its matching pixel coordinates.
            # The new feedback anchors the next motion to the lifted pose.
            current_registration=o['scene_registration']
            o={**previous,'before':o['before'],'after':o['after']}
            o['scene_registration']={**current_registration,
                'nominal_scene_geometry':previous['scene_registration']['nominal_scene_geometry'],
                'stationary_box_reuse':{**evidence,'reference':str(previous_registration),
                                      'new_observation':str(path.resolve())}}
            out=directory/(path.stem+'_registered.json')
            out.write_text(json.dumps(o,ensure_ascii=False,indent=2),encoding='utf-8')
            print('抬高后已核对板和盒面，沿用抬高前定位：',out,flush=True)
            return out
        if not saved_plane and outline.get('frame_edge')=='upper':
            corners=np.asarray(fit['corners_pixel'])
            edge=int(np.argmin((corners[:,1]+np.roll(corners[:,1],-1))/2))
            pixels=corners[[edge,(edge+1)%4]].tolist()
            raise ObservationBoxEdgeUnclear(f'{message}；疑似药盒上沿出框',best['T_base_board_cm'],pixels)
        if not saved_plane and outline.get('frame_edge')=='lower':
            raise ObservationBoxEdgeUnclear(f'{message}；药盒下沿证据不足',best['T_base_board_cm'])
        raise ValueError(message)
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
    print('新场景注册：',out,'本帧板误差px：',cal['scene_registration']['board_pnp_rmse_px'],'目标中心cm：',o['geometry']['selected']['top_center_base_cm'])
    return out

def undirected_angle_deg(axis,reference):
    scale=np.linalg.norm(axis)*np.linalg.norm(reference)
    if not np.isfinite(scale) or scale<=0:raise ValueError('无法计算对齐角度')
    return float(np.degrees(np.arccos(np.clip(abs(axis@reference)/scale,0,1))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('observation',type=Path)
    p.add_argument('--motion-profile',type=Path)
    p.add_argument('--board-profile',type=Path)
    a=p.parse_args()
    register(a.observation,motion=load_motion_profile(a.motion_profile),board_profile=a.board_profile)
