"""Current-box/fixed-board visual feedback. No hardware writes.

Camera pose comes from the fixed board. Box pose comes from its measured rim.
"""
import json
import math
from pathlib import Path
import cv2
import numpy as np
from check_grasp_board import GRASP_BOARD_PROFILE, LARGE_GRASP_BOARD_PROFILE, ROOT, make_board
from fit_grasp_handeye import transform,discrepancy,robot_frame
from jpeg_integrity import check_jpeg
from sample_grasp_handeye import nominal_model
from box_rim_geometry import RimEvidence,face_corners,lift_rim_pose,refine_grounded_rim
from board_marker_pose import marker_plane_candidates_with_contrast

# Frozen commissioning image-consistency policy. These pixel residuals are not
# bounds on physical TCP accuracy; the uncertain TCP retains clearance and is
# never certified by an image-fit threshold alone.
QUALITY_POLICY={'version':'independent_box_rim_v1','board_refined_rmse_px':1.0}


def inverse(T):return np.linalg.inv(T)


class GraspVisualObserver:
    def __init__(self,observation_path,tcp_path):
        self.observation_path=Path(observation_path).resolve()
        self.observation=json.loads(Path(observation_path).read_text(encoding='utf-8'))
        self.tcp=json.loads(Path(tcp_path).read_text(encoding='utf-8'))
        self.cal=json.loads(Path(self.observation['calibration']).read_text(encoding='utf-8'))['candidates'][0]
        self.Y=np.array(self.cal['T_base_board_cm'])
        self.X=np.array(self.cal['T_mount_camera_cm'])
        self.model=nominal_model()
        data=json.loads((ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json').read_text(encoding='utf-8'))
        self.K,self.D=np.array(data['camera_matrix_px']),np.array(data['distortion_coefficients'])
        profile=self.observation.get('scene_registration',{}).get('board_profile')
        if profile is None:
            profile=json.loads(GRASP_BOARD_PROFILE.read_text(encoding='utf-8'))
        self.large_board=profile.get('board_id')==LARGE_GRASP_BOARD_PROFILE.stem
        self.board=make_board(profile)
        self.dictionary=self.board.getDictionary()
        dp=cv2.aruco.DetectorParameters();dp.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(self.dictionary,dp)
        self.refs=[];self.last_camera=None;self.last_object=None
        geometry=self.observation['geometry']
        selected=geometry['selected']
        self.dimensions=geometry['assumed_box_dimensions_cm']
        axis=np.asarray(selected['length_axis_base'])
        normal=self.Y[:3,2]
        self.template=transform(np.column_stack((axis,np.cross(normal,axis),normal)),selected['top_center_base_cm'])
        self.add_reference(Path(self.observation['image']),selected['corners_pixel'],camera=np.array(geometry['T_base_camera_cm']))
        self.corner_local=face_corners(self.dimensions)
        self.board_maps=[]
        self.sift=cv2.SIFT_create(nfeatures=2000)
        for ref in self.refs:
            self.build_board_map(ref)
        # Recovery of this unchanged scene only: never replay commands from the
        # journal. Reuse only previously accepted, grounded image observations.
        history=[]
        for path in sorted((ROOT/'vision_grasp_results').glob('visual_approach_*.json')):
            saved=json.loads(path.read_text(encoding='utf-8'))
            if Path(saved.get('source','')).resolve()!=Path(observation_path).resolve() or saved.get('gripper_close_sent'):
                continue
            for packet in saved.get('observations',[]):
                visual=packet.get('visual')
                if visual and visual.get('object_pose_basis')=='known_stationary_support_plane':
                    history.append((packet['capture']['image_path'],visual))
        for path,visual in history[-4:]:
            C=np.array(visual['T_camera_object']);camera=np.array(visual['T_base_camera'])
            q=cv2.projectPoints(self.corner_local,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],self.K,self.D)[0].reshape(-1,2)
            self.add_reference(path,q.tolist(),camera)
            self.build_board_map(self.refs[0])
        self.board_maps=self.board_maps[-6:]

    def restore_grounded_journal(self,journal,seen=None):
        """Restore accepted image/metric keyframes across process boundaries.

        This never replays hardware commands or treats a recorded pose as a
        current observation. A new capture is still required before motion.
        """
        if Path(journal['source']).resolve()!=self.observation_path or journal.get('gripper_close_sent'):
            raise ValueError('关键帧不属于当前未闭爪场景')
        if any(c.get('task',{}).get('status')!='reached' for c in journal.get('commands',[])):
            raise ValueError('历史存在未知运动，不能恢复关键帧用于接续')
        seen=set() if seen is None else seen
        if journal.get('seed_approach'):
            path=Path(journal['seed_approach']).resolve()
            if path in seen or len(seen)>=8:raise ValueError('关键帧接续链无效或过长')
            seen.add(path);self.restore_grounded_journal(json.loads(path.read_text(encoding='utf-8')),seen)
        frames=[]
        if journal.get('hover_handoff',{}).get('visual'):frames.append(journal['hover_handoff']['visual'])
        frames.extend(p['visual'] for p in journal.get('observations',[]) if p.get('visual'))
        for visual in frames:
            if visual.get('object_pose_basis')!='known_stationary_support_plane':raise ValueError('不能把已离面物体当作固定场景')
            C=np.array(visual['T_camera_object']);camera=np.array(visual['T_base_camera'])
            q=cv2.projectPoints(self.corner_local,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],self.K,self.D)[0].reshape(-1,2)
            self.add_reference(Path(visual['image']),q.tolist(),camera)
            self.build_board_map(self.refs[0]);self.board_maps=self.board_maps[-6:]
            self.last_camera,self.last_object=camera,C

    def build_board_map(self,ref):
        camera=ref['metric_camera'];C=inverse(camera)@self.Y
        width,height=np.array(self.board.getChessboardSize())*self.board.getSquareLength()
        corners=np.array([[0,0,0],[width,0,0],[width,height,0],[0,height,0]],np.float64)
        q=cv2.projectPoints(corners,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],self.K,self.D)[0].reshape(-1,2)
        mask=np.zeros_like(ref['gray']);cv2.fillConvexPoly(mask,np.rint(q).astype(np.int32),255)
        object_mask=np.zeros_like(mask);cv2.fillConvexPoly(object_mask,np.rint(ref['corners']).astype(np.int32),255)
        object_mask=cv2.dilate(object_mask,np.ones((13,13),np.uint8))
        mask[object_mask>0]=0;mask[420:]=0
        keys,descriptors=self.sift.detectAndCompute(ref['gray'],mask)
        if descriptors is None or len(keys)<20:return
        pixels=np.array([k.pt for k in keys],np.float64)
        uv=cv2.undistortPoints(pixels.reshape(-1,1,2),self.K,self.D).reshape(-1,2)
        board_camera=inverse(self.Y)@camera
        rays=(board_camera[:3,:3]@np.column_stack((uv,np.ones(len(uv)))).T).T
        depth=-board_camera[2,3]/rays[:,2]
        points=board_camera[:3,3]+rays*depth[:,None];points[:,2]=0
        self.board_maps.append({'keys':keys,'descriptors':descriptors,'points':points,'reference':ref['path']})

    def add_reference(self,path,corners,camera=None,joints=None):
        im=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),0)
        if im is None:raise ValueError('关键帧无法解码')
        if camera is None:
            candidates,_=self.board_candidates(im)
            if joints is not None:
                expected=robot_frame(joints,self.model,'pitch_tool')@self.X
                candidates=[r for r in candidates if discrepancy(r[0],expected)[1]<12]
            if not candidates:return
            camera=min(candidates,key=lambda row:row[1])[0]
        self.refs.insert(0,{'path':str(path),'gray':im,'corners':corners,'metric_camera':camera})
        self.refs=self.refs[:4]

    def board_candidates(self,gray,image_path=None):
        poses,metadata=marker_plane_candidates_with_contrast(
            gray,self.K,self.D,self.board,self.detector,allow_three=True,
            allow_isolated_outlier=self.large_board,allow_wide_inlier_support=self.large_board)
        result=[(self.Y@inverse(C),error) for C,error in poses]
        if result:
            return result,metadata
        if getattr(self,'board_maps',None):
            keys,desc=self.sift.detectAndCompute(gray,None)
            if desc is not None:
                for ref in self.board_maps:
                    if image_path is not None and Path(ref['reference']).resolve()==Path(image_path).resolve():
                        continue
                    matcher=cv2.BFMatcher();forward=matcher.knnMatch(ref['descriptors'],desc,k=2)
                    reverse={m.queryIdx:m.trainIdx for m in matcher.match(desc,ref['descriptors'])}
                    good=[m[0] for m in forward if len(m)==2 and m[0].distance<.7*m[1].distance and reverse.get(m[0].trainIdx)==m[0].queryIdx]
                    unique={}
                    for m in sorted(good,key=lambda m:m.distance):
                        unique.setdefault(tuple(np.round(ref['points'][m.queryIdx,:2],2)),m)
                    good=list(unique.values())
                    if len(good)<16:continue
                    obj=np.array([ref['points'][m.queryIdx] for m in good],np.float64)
                    image=np.array([keys[m.trainIdx].pt for m in good],np.float64)
                    undistorted=cv2.undistortPoints(image.reshape(-1,1,2),self.K,self.D,P=self.K).reshape(-1,2)
                    _,mask=cv2.findHomography(obj[:,:2],undistorted,cv2.RANSAC,1.5)
                    if mask is None:continue
                    keep=mask.ravel().astype(bool)
                    if keep.sum()<16 or keep.mean()<.7:continue
                    obj,image=obj[keep],image[keep]
                    if cv2.contourArea(cv2.convexHull(obj[:,:2].astype(np.float32)))<2:continue
                    solved=cv2.solvePnPGeneric(obj,image,self.K,self.D,flags=cv2.SOLVEPNP_IPPE)
                    for rv,tv in zip(solved[1],solved[2]):
                        C=transform(cv2.Rodrigues(rv)[0],tv)
                        px=cv2.projectPoints(obj,rv,tv,self.K,self.D)[0].reshape(-1,2)
                        rmse=float(np.sqrt(np.mean(np.sum((px-image)**2,axis=1))))
                        if rmse<=1.0 and np.all((C[:3,:3]@obj.T+C[:3,3:])[2]>0):
                            result.append((self.Y@inverse(C),rmse))
                            metadata.setdefault('texture_candidate_evidence',[]).append({
                                'pose_support':'fixed_board_texture',
                                'texture_reference':ref['reference'],
                                'measured_points_board_cm':obj.tolist(),
                                'measured_points_pixel':image.tolist(),
                                'measurement_weight':1,
                            })
                    metadata.setdefault('texture_support',[]).append({'reference':ref['reference'],'inliers':int(keep.sum())})
        return result,metadata

    def stationary_board_pose(self,gray,joints,reference):
        visual,previous_joints=reference
        if not visual['board'].get('decoded_pose_accepted'):
            return None
        if any(abs(joints[j]-previous_joints[j])>2 for j in joints):
            return None
        previous=cv2.imdecode(np.frombuffer(Path(visual['image']).read_bytes(),np.uint8),0)
        if previous is None:
            return None
        layout={int(i):p.mean(axis=0)[:2] for i,p in zip(self.board.getIds().ravel(),self.board.getObjPoints())}
        def corners(image):
            points,ids,_=self.detector.detectMarkers(image)
            return {} if ids is None else {int(i):p.reshape(4,2) for i,p in zip(ids.ravel(),points) if int(i) in layout}
        first,second=corners(previous),corners(gray)
        common=sorted(first.keys() & second.keys())
        if len(common)<4:
            return None
        hull=cv2.convexHull(np.array([layout[i] for i in common],np.float32))
        if cv2.contourArea(hull)<2:
            return None
        shifts=np.concatenate([second[i]-first[i] for i in common])
        distances=np.linalg.norm(shifts,axis=1)
        rmse=float(np.sqrt(np.mean(distances**2)))
        if rmse>2. or distances.max()>3.5:
            return None
        camera=np.array(visual['T_base_camera'])
        return camera,visual['board_rmse_px'],{
            'pose_support':'stationary_hold_board_corners',
            'marker_ids':common,'marker_count':len(common),
            'stationary_reference_image':visual['image'],
            'corner_shift_rmse_px':rmse,'corner_shift_max_px':float(distances.max()),
        }

    def estimate_camera(self,gray,path,joints,stationary_reference=None):
        expected=self.last_camera if self.last_camera is not None else robot_frame(joints,self.model,'pitch_tool')@self.X
        stationary=(self.stationary_board_pose(gray,joints,stationary_reference)
                    if stationary_reference is not None else None)
        if stationary is None:
            board_cameras,metadata=self.board_candidates(gray,image_path=path)
        else:
            camera,error,metadata=stationary
            board_cameras=[(camera,error)]
        possible=[(camera,error,discrepancy(camera,expected)[1],index)
                  for index,(camera,error) in enumerate(board_cameras)
                  if discrepancy(camera,expected)[1]<12]
        if not possible:raise ValueError('固定参照板未提供可靠的独立相机位姿：'+str(metadata))
        camera,board_rmse,_,index=min(possible,key=lambda r:r[1]+.1*r[2])
        texture_evidence=metadata.pop('texture_candidate_evidence',None)
        if texture_evidence is not None:
            metadata.update(texture_evidence[index])
        return camera,board_rmse,metadata

    def estimate(self,path,joints,held=False,*,stationary_reference=None):
        integrity=check_jpeg(path)
        if not integrity['valid']:raise ValueError('JPEG不完整：'+str(integrity))
        gray=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),0)
        if gray is None:raise ValueError('近图无法解码')
        camera,board_rmse,metadata=self.estimate_camera(gray,path,joints,stationary_reference if held else None)
        evidence=RimEvidence(gray,self.K,self.D,self.corner_local)
        if held:
            if self.last_object is None:
                raise ValueError('试提验收缺少夹取前的几何位姿基准')
            candidates=lift_rim_pose(evidence,self.last_object)
            candidates=[(score,error,pose) for score,error,pose in candidates
                        if discrepancy(pose,self.last_object)[1]<=15
                        and discrepancy(pose,self.last_object)[0]<=1.5
                        and self.Y[:3,2]@(camera@pose)[:3,2]>=math.cos(math.radians(15))]
            if not candidates:
                raise ValueError('试提图未测得可靠的盒边位姿，无法确认离面')
            score,obj_rmse,C=candidates[0]
            if any(other_score<=score+.12 and (shift>.35 or angle>5)
                   for other_score,_,pose in candidates[1:] for shift,angle in [discrepancy(C,pose)]):
                raise ValueError('试提图盒边支持多个不同位姿，无法确认离面')
            tracked={'status':'visible_rim_measured','method':'free_box_rim_lines'}
            pose_estimator='free_box_rim_lines'
        else:
            prior_world=(self.last_camera@self.last_object
                         if self.last_camera is not None and self.last_object is not None else self.template)
            fit=refine_grounded_rim(evidence,inverse(camera)@self.template,allow_cropped=True,
                                    prior_pose=inverse(camera)@prior_world)
            if fit is None:
                raise ValueError('近图可见盒边证据不足或存在多个相近候选，无法可靠定位药盒')
            C,rim=fit
            obj_rmse=rim['mean_edge_distance_px']
            tracked={'status':'visible_rim_measured','edge_coverage':rim['coverage'],
                     'edge_mean_distance_px':rim['distances'],
                     'candidate_selection':rim.get('candidate_selection','lowest_edge_distance'),
                     'prior_translation_delta_cm':rim.get('prior_translation_delta_cm'),
                     'near_tied_candidate_count':rim.get('near_tied_candidate_count',0)}
            pose_estimator='independent_camera_and_visible_box_rim'
        world=camera@C;obj_board=inverse(self.Y)@world
        tcp_camera=np.array(self.tcp['center_camera_cm']);tcp_base=camera[:3,:3]@tcp_camera+camera[:3,3]
        tcp_board=inverse(self.Y)@np.r_[tcp_base,1]
        bottom=self.corner_local.copy();bottom[:,2]=self.dimensions[2]
        bottom_board=(obj_board[:3,:3]@bottom.T+obj_board[:3,3:]).T
        face_px=cv2.projectPoints(self.corner_local,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],self.K,self.D)[0].reshape(-1,2)
        record={'image':str(path),'jpeg_integrity':integrity,'board':metadata,'board_rmse_px':board_rmse,
            'object_rmse_px':obj_rmse,'tracking':tracked,'reference_used':None,
            'T_base_camera':camera.tolist(),'T_camera_object':C.tolist(),'T_board_object':obj_board.tolist(),
            'tcp_base_cm':tcp_base.tolist(),'tcp_board_cm':tcp_board[:3].tolist(),
            'top_center_base_cm':world[:3,3].tolist(),'top_height_cm':-float(obj_board[2,3]),
            'minimum_box_bottom_height_cm':float(np.min(-bottom_board[:,2])),
            'tcp_estimate_is_approximate':True,'for_accuracy_claim':False}
        record['quality_policy']=QUALITY_POLICY
        record['pose_estimator']=pose_estimator
        record['object_error_metric']='observed_rim_line_residual_px' if held else 'mean_visible_rim_pixel_distance'
        if 'measured_points_board_cm' in metadata:
            points=np.array(metadata['measured_points_board_cm']);Cboard=inverse(camera)@self.Y
            projection=cv2.projectPoints(points,cv2.Rodrigues(Cboard[:3,:3])[0],Cboard[:3,3],self.K,self.D)[0].reshape(-1,2)
            record['final_board_fit_rmse_px']=float(np.sqrt(np.mean(np.sum((projection-np.array(metadata['measured_points_pixel']))**2,axis=1))))
        record['object_pose_basis']='free_pose_of_measured_rim' if held else 'known_stationary_support_plane'
        self.last_camera,self.last_object=camera,C
        if not held:
            self.add_reference(path,face_px.tolist(),camera)
            if metadata.get('decoded_pose_accepted'):
                self.build_board_map(self.refs[0])
                self.board_maps=self.board_maps[-6:]
        return record
