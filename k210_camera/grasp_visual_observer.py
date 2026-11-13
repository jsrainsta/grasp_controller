"""Current-box/fixed-board visual feedback. No hardware writes.

Known same box only. Uses real decoded marker corners for the stationary plane,
and observed texture matches for a partly cropped box, without inventing corners.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from check_grasp_board import ROOT
from fit_grasp_handeye import transform,discrepancy,robot_frame
from jpeg_integrity import check_jpeg
from sample_grasp_handeye import nominal_model
from track_box_reference import track
from board_marker_pose import marker_plane_candidates_with_contrast

# Frozen commissioning image-consistency policy. These pixel residuals are not
# bounds on physical TCP accuracy; the uncertain TCP retains clearance and is
# never certified by an image-fit threshold alone.
QUALITY_POLICY={'version':'commissioning_v1','board_refined_rmse_px':1.0,
                'ground_texture_rmse_px':3.0,'free_object_rmse_px':1.5}


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
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board=cv2.aruco.CharucoBoard((7,5),1.44,.96,self.dictionary)
        dp=cv2.aruco.DetectorParameters();dp.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(self.dictionary,dp)
        self.refs=[];self.last_camera=None;self.last_object=None
        selected=self.observation['geometry']['selected']
        self.template=transform(np.column_stack((selected['length_axis_base'],selected['width_axis_base'],self.Y[:3,2])),selected['top_center_base_cm'])
        # A new arrival/placement must not inherit old stationary-object maps.
        historical_scene=Path(observation_path).name=='photo_20261105_120734_974932_seq1_396c5888_d558ca46_commissioning_boardanchor.json'
        if historical_scene:
            near=json.loads((ROOT/'vision_grasp_results/relocalized_near_keyframe_20261105_01.json').read_text(encoding='utf-8'))
            near_pose=json.loads((ROOT/'vision_grasp_results/hover_20261105_201456_0447b0.json').read_text(encoding='utf-8'))['final']['joint_median']
            self.add_reference(ROOT/'vision_grasp_images/20261105'/near['current'],near['tracking']['inferred_face_corners_pixel'],joints=near_pose)
        self.add_reference(Path(self.observation['image']),self.observation['geometry']['selected']['corners_pixel'],camera=np.array(self.observation['geometry']['T_base_camera_cm']))
        self.corner_local=np.array([[-5.25,-1.7,0],[5.25,-1.7,0],[5.25,1.7,0],[-5.25,1.7,0]],np.float64)
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
        corners=np.array([[0,0,0],[10.08,0,0],[10.08,7.2,0],[0,7.2,0]],np.float64)
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
        poses,metadata=marker_plane_candidates_with_contrast(gray,self.K,self.D,self.board,self.detector,allow_three=True)
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

    def plane_points(self,pixels,camera):
        uv=cv2.undistortPoints(np.array(pixels,np.float64).reshape(-1,1,2),self.K,self.D).reshape(-1,2)
        rays=(camera[:3,:3]@np.column_stack((uv,np.ones(len(uv)))).T).T
        n=self.Y[:3,2];origin=self.Y[:3,3]-n*2.5
        depths=((origin-camera[:3,3])@n)/(rays@n)
        if np.any(depths<=0):raise ValueError('顶面射线不在相机前方')
        return camera[:3,3]+rays*depths[:,None]

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

    def estimate(self,path,joints,held=False,*,stationary_reference=None):
        integrity=check_jpeg(path)
        if not integrity['valid']:raise ValueError('JPEG不完整：'+str(integrity))
        gray=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),0)
        expected=self.last_camera if self.last_camera is not None else robot_frame(joints,self.model,'pitch_tool')@self.X
        stationary=(self.stationary_board_pose(gray,joints,stationary_reference)
                    if held and stationary_reference is not None else None)
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
        attempts=[];independent_ground_attempts=[];diagnostics=[]
        for ref in self.refs:
            # Predict magnification from the measured camera pose instead of
            # assuming the same scale at the overview and near-grasp views.
            nominal_face=self.last_object if held and self.last_object is not None else inverse(camera)@self.template
            projected=cv2.projectPoints(self.corner_local,cv2.Rodrigues(nominal_face[:3,:3])[0],nominal_face[:3,3],self.K,self.D)[0].reshape(-1,2)
            area=abs(cv2.contourArea(np.array(ref['corners'],np.float32)))
            scale=abs(cv2.contourArea(projected.astype(np.float32)))/max(area,1.)
            result=track(ref['gray'],gray,ref['corners'],scale_bounds=(scale*.6,scale*1.6),ratio_threshold=.7,coverage_basis='common_visible',current_face_hint=projected)
            if result['status']!='tracked_face_for_review':
                diagnostics.append({'reference':ref['path'],'tracking_reason':result.get('reason'),
                    'match_count':result.get('match_count'),'inlier_count':result.get('inlier_count'),
                    'visible_coverage':result.get('reference_visible_face_coverage')})
                continue
            world_points=self.plane_points(result['inlier_source_pixel'],ref['metric_camera'])
            obj=(inverse(self.template)[:3,:3]@world_points.T+inverse(self.template)[:3,3:]).T
            pixels=np.array(result['inlier_current_pixel'],np.float64)
            if not held:
                # Independent current-view estimator. Do not force camera
                # measurements to fit a possibly biased world map learned at
                # a distant pose. Require strong texture, decoded marker
                # support, and metric shape agreement in the current view.
                if (metadata.get('decoded_pose_accepted')
                        and result['inlier_count']>=20 and result['median_inlier_error_px']<=1.0
                        and 'measured_points_board_cm' in metadata):
                    board_points=np.array(metadata['measured_points_board_cm'])
                    Cboard=inverse(camera)@self.Y
                    board_projection=cv2.projectPoints(board_points,cv2.Rodrigues(Cboard[:3,:3])[0],Cboard[:3,3],self.K,self.D)[0].reshape(-1,2)
                    independent_board_error=float(np.sqrt(np.mean(np.sum((board_projection-np.array(metadata['measured_points_pixel']))**2,axis=1))))
                    corners_world=self.plane_points(result['inferred_face_corners_pixel'],camera)
                    edges=np.roll(corners_world,-1,axis=0)-corners_world;lengths=np.linalg.norm(edges,axis=1)
                    long=float((lengths[0]+lengths[2])/2);short=float((lengths[1]+lengths[3])/2)
                    orth=float(abs(edges[0]@edges[1]/lengths[0]/lengths[1]))
                    shape={'length_cm':long,'width_cm':short,'orthogonality_error':orth,'independent_marker_rmse_px':independent_board_error}
                    diagnostics.append({'current_view_shape':shape,'reference':ref['path']})
                    # Use the same center-error ceiling as the already
                    # accepted decoded pose, plus the independent shape gates.
                    if independent_board_error<=.6 and abs(long/10.5-1)<=.08 and abs(short/3.4-1)<=.08 and orth<=.08:
                        axis=edges[0]-edges[2];axis/=np.linalg.norm(axis)
                        rotation=np.column_stack((axis,np.cross(self.Y[:3,2],axis),self.Y[:3,2]))
                        independent_world=transform(rotation,corners_world.mean(0));independent_C=inverse(camera)@independent_world
                        independent_ground_attempts.append((result['median_inlier_error_px'],independent_C,
                            result['median_inlier_error_px'],result,ref,camera,shape))
                # Grounded object texture and measured fixed-board points are
                # two depth planes. Refine the current camera against both,
                # retaining separate residual gates. This is not allowed once
                # the box is lifted, when the object is no longer stationary.
                camera_for_ref=camera
                if 'measured_points_board_cm' in metadata:
                    board_points=np.array(metadata['measured_points_board_cm'])
                    board_world=(self.Y[:3,:3]@board_points.T+self.Y[:3,3:]).T
                    board_pixels=np.array(metadata['measured_points_pixel'])
                    initial=inverse(camera)
                    weight=metadata['measurement_weight']
                    # A dense box match can overpower a sparse fixed-board map.
                    # Keep both residual limits and try stronger board anchoring.
                    weights=(weight,4*weight,16*weight) if metadata.get('pose_support')=='fixed_board_texture' else (weight,)
                    for fit_weight in weights:
                        combined_obj=np.concatenate((np.repeat(board_world,fit_weight,axis=0),world_points))
                        combined_pixels=np.concatenate((np.repeat(board_pixels,fit_weight,axis=0),pixels))
                        rv,tv=cv2.solvePnPRefineLM(combined_obj,combined_pixels,self.K,self.D,
                            cv2.Rodrigues(initial[:3,:3])[0],initial[:3,3].reshape(3,1))
                        Cfit=transform(cv2.Rodrigues(rv)[0],tv)
                        projected_board=cv2.projectPoints(board_world,rv,tv,self.K,self.D)[0].reshape(-1,2)
                        fit_board_error=float(np.sqrt(np.mean(np.sum((projected_board-board_pixels)**2,axis=1))))
                        if fit_board_error<=QUALITY_POLICY['board_refined_rmse_px']:
                            break
                    if fit_board_error>QUALITY_POLICY['board_refined_rmse_px']:
                        diagnostics.append({'board_refinement_rmse_px':fit_board_error,'reference':ref['path']})
                        continue
                    camera_for_ref=inverse(Cfit)
                Hcenter=cv2.getPerspectiveTransform(self.corner_local[:,:2].astype(np.float32),np.array(ref['corners'],np.float32))
                center_px=cv2.perspectiveTransform(np.zeros((1,1,2),np.float32),Hcenter)
                center_px=cv2.perspectiveTransform(center_px,np.array(result['homography_reference_to_current']))
                center_world=self.plane_points(center_px.reshape(1,2),camera_for_ref)[0]
                world=transform(self.template[:3,:3],center_world)
                C=inverse(camera_for_ref)@world
                Cref=inverse(camera_for_ref)
                pred=cv2.projectPoints(world_points,cv2.Rodrigues(Cref[:3,:3])[0],Cref[:3,3],self.K,self.D)[0].reshape(-1,2)
                error=float(np.sqrt(np.mean(np.sum((pred-pixels)**2,axis=1))))
                diagnostics.append({'ground_map_rmse_px':error,'inliers':result.get('inlier_count'),'reference':ref['path']})
                if error<=QUALITY_POLICY['ground_texture_rmse_px']:
                    attempts.append((error,C,error,result,ref,camera_for_ref))
                continue
            # Metric points were learned on the known support plane before
            # closure; after lifting their free pose is measured, not assumed.
            obj[:,2]=0
            solved=cv2.solvePnPGeneric(obj,pixels,self.K,self.D,flags=cv2.SOLVEPNP_IPPE)
            for rv,tv in zip(solved[1],solved[2]):
                C=transform(cv2.Rodrigues(rv)[0],tv)
                if np.any((C[:3,:3]@obj.T+C[:3,3:])[2]<=0):continue
                predicted=cv2.projectPoints(obj,rv,tv,self.K,self.D)[0].reshape(-1,2)
                error=float(np.sqrt(np.mean(np.sum((predicted-pixels)**2,axis=1))))
                world=camera@C;board_obj=inverse(self.Y)@world
                height=-float(board_obj[2,3])
                tilt=float(np.degrees(np.arccos(np.clip(board_obj[2,2],-1,1))))
                diagnostics.append({'rmse_px':error,'height_cm':height,'tilt_deg':tilt,
                                    'inliers':result.get('inlier_count'),
                                    'reference':ref['path']})
                if error>QUALITY_POLICY['free_object_rmse_px'] or tilt>15:continue
                if not held and not 1.8<=height<=3.2:continue
                consistency=0 if self.last_object is None else discrepancy(C,self.last_object)[1]
                score=error+.08*tilt+(.02*consistency if held else abs(height-2.5))
                attempts.append((score,C,error,result,ref,camera))
        independent_shape=None
        if not attempts and independent_ground_attempts:
            selected=min(independent_ground_attempts,key=lambda r:r[0]);independent_shape=selected[6];attempts=[selected[:6]]
        if not attempts:raise ValueError('盒面真实匹配点未形成可靠位姿：'+json.dumps(diagnostics,ensure_ascii=False))
        _,C,obj_rmse,tracked,ref,camera=min(attempts,key=lambda r:r[0])
        world=camera@C;obj_board=inverse(self.Y)@world
        tcp_camera=np.array(self.tcp['center_camera_cm']);tcp_base=camera[:3,:3]@tcp_camera+camera[:3,3]
        tcp_board=inverse(self.Y)@np.r_[tcp_base,1]
        bottom=self.corner_local.copy();bottom[:,2]=2.5
        bottom_board=(obj_board[:3,:3]@bottom.T+obj_board[:3,3:]).T
        face_px=cv2.projectPoints(self.corner_local,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],self.K,self.D)[0].reshape(-1,2)
        record={'image':str(path),'jpeg_integrity':integrity,'board':metadata,'board_rmse_px':board_rmse,
            'object_rmse_px':obj_rmse,'tracking':tracked,'reference_used':ref['path'],
            'T_base_camera':camera.tolist(),'T_camera_object':C.tolist(),'T_board_object':obj_board.tolist(),
            'tcp_base_cm':tcp_base.tolist(),'tcp_board_cm':tcp_board[:3].tolist(),
            'top_center_base_cm':world[:3,3].tolist(),'top_height_cm':-float(obj_board[2,3]),
            'minimum_box_bottom_height_cm':float(np.min(-bottom_board[:,2])),
            'tcp_estimate_is_approximate':True,'for_accuracy_claim':False}
        record['quality_policy']=QUALITY_POLICY
        record['pose_estimator']='independent_current_marker_camera_and_tracked_plane' if independent_shape is not None else 'two_plane_refinement' if not held else 'free_object_landmark_pnp'
        if independent_shape is not None:
            record['current_view_shape_validation']=independent_shape
            record['object_error_metric']='homography_inlier_median_px; not metric-map reprojection RMSE'
        if 'measured_points_board_cm' in metadata:
            points=np.array(metadata['measured_points_board_cm']);Cboard=inverse(camera)@self.Y
            projection=cv2.projectPoints(points,cv2.Rodrigues(Cboard[:3,:3])[0],Cboard[:3,3],self.K,self.D)[0].reshape(-1,2)
            record['final_board_fit_rmse_px']=float(np.sqrt(np.mean(np.sum((projection-np.array(metadata['measured_points_pixel']))**2,axis=1))))
        record['object_pose_basis']='free_pose_of_ground_learned_landmarks' if held else 'known_stationary_support_plane'
        self.last_camera,self.last_object=camera,C
        if not held:
            self.add_reference(path,face_px.tolist(),camera)
            if metadata.get('decoded_pose_accepted'):
                self.build_board_map(self.refs[0])
                self.board_maps=self.board_maps[-6:]
        return record
