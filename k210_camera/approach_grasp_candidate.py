"""Commissioning hover only, 5cm nominal clearance; no jaw/roll/contact commands."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from check_grasp_board import ROOT
from collect_grasp_reference import read_arm
from fit_grasp_handeye import robot_frame,preview_wrist_alignment
from jpeg_integrity import check_jpeg
from plan_fixed_box_grasp import ik
from receive_photo import receive_photos
from sample_grasp_handeye import call,nominal_model
from track_box_reference import track
from grasp_start_state import require_start_state,start_state_error
from grasp_motion_profile import load_motion_profile
from tcp_pose_planner import plan_step

def coarse_feedback(base):
    """One fresh joint read between already-confirmed coarse segments."""
    status=call(base,'status')
    if status.get('connection')!='connected' or status.get('active_task_id') is not None or status.get('upload_in_progress') or status.get('limits_require_recheck'):
        raise RuntimeError('预接近段间状态不允许继续')
    p=call(base,'positions?refresh=true')
    if p.get('fresh') is not True or p.get('source')!='serial':raise RuntimeError('预接近缺少新串口反馈')
    return {'status':{'ok':True,'mode':'serial','data':status},
            'joint_median':{str(v['id']):v['position'] for v in p['positions']}}

class PregraspClearanceError(ValueError):
    pass


def path_margin(start,end,model,center,up,limits,tcp_offset=None,minimum_top_clearance=None):
    for j in ('3','4','5','6'):
        if not limits[int(j)][0]<=end[j]<=limits[int(j)][1]:
            raise ValueError('目标超出当前关节限位')
    gaps=[]
    for fraction in np.linspace(0,1,41):
        p={int(j):round(start[j]+(end[j]-start[j])*fraction) for j in start}
        # Feedback can be a few counts outside a commanded endpoint limit.
        # Accept only the existing small excursion while returning inside;
        # every new target above remains strictly inside configured limits.
        for j in ('3','4','5','6'):
            lo,hi=limits[int(j)]
            if not min(lo,start[j])<=p[int(j)]<=max(hi,start[j]) or not lo-10<=p[int(j)]<=hi+10:
                raise ValueError('路径超出关节反馈边界')
        pose=model.forward(p)
        point=np.array([pose['x_cm'],pose['y_cm'],pose['z_cm']])
        if tcp_offset is not None:
            point=robot_frame({str(j):v for j,v in p.items()},model,'pitch_tool')[:3,:3]@tcp_offset+point
        gap=float((point-center)@up)
        minimum=(2.8 if tcp_offset is not None else 4.7) if minimum_top_clearance is None else minimum_top_clearance
        if gap<minimum:
            raise PregraspClearanceError(f'预接近路径名义余量不足（{gap:.3f} cm，要求 {minimum:.3f} cm）')
        gaps.append(gap)
    return min(gaps)

def pregrasp_candidates(model,start,center,up,limits,tcp_offset,motion,height,rejections,*,split_descent=False):
    point=center+up*(height-2.5 if tcp_offset is not None else 5)
    candidates=[]
    pitches = (-90,-85,-80,-75,-70,-65,motion['preferred_nominal_pitch_deg'])
    if motion.get('adaptive_pitch_search'):
        low,high=motion['nominal_pitch_range_deg']
        pitches=(*pitches,low,high,*range(math.ceil(low),math.floor(high)+1))
    pitches=dict.fromkeys(pitches)
    for pitch in pitches:
        if not motion['nominal_pitch_range_deg'][0]<=pitch<=motion['nominal_pitch_range_deg'][1]:continue
        goal=point.copy()
        if tcp_offset is not None:
            yaw=math.atan2(point[1],point[0]);theta=math.radians(pitch)
            for iteration in range(5):
                c,s,ct,st=math.cos(yaw),math.sin(yaw),math.cos(theta),math.sin(theta)
                rotation=np.array([[c*ct,-s,-c*st],[s*ct,c,-s*st],[st,0,ct]])
                goal=point-rotation@tcp_offset
                yaw=math.atan2(goal[1],goal[0])
        for solution in ik(goal,pitch,model.to_config()):
            end={**start,**solution['illustrative_joint_positions_not_for_execution']}
            try:margin=path_margin(start,end,model,center,up,limits,tcp_offset,motion['pregrasp_min_top_clearance_cm'])
            except ValueError as error:
                reason='预接近路径名义余量不足' if isinstance(error,PregraspClearanceError) else str(error)
                rejections[reason]=rejections.get(reason,0)+1
                continue
            joint_margin=min(min(end[j]-limits[int(j)][0],limits[int(j)][1]-end[j]) for j in ('3','4','5','6'))
            final_plan=None
            if motion.get('whole_path_precheck') and tcp_offset is not None:
                M=robot_frame(end,model,'pitch_tool');pre_tcp=M[:3,:3]@tcp_offset+M[:3,3]
                final=center+up*(motion['grasp_region']['tcp_height_target_cm']-2.5)
                try:
                    height_from_joints=float((pre_tcp-center)@up)+2.5
                    if split_descent:
                        # Check two shorter nominal steps, as the vision-guided
                        # approach can move in increments instead of one jump.
                        first=plan_step(model,end,(final-pre_tcp)*.5,pitch,tcp_offset,limits,
                            height_from_joints,up,motion['minimum_nominal_pad_height_cm'])
                        mid={**end,**first['expected_joint_positions']}
                        mid_tcp=pre_tcp+np.array(first['predicted_delta_cm'])
                        final_plan=plan_step(model,mid,final-mid_tcp,pitch,tcp_offset,limits,
                            float((mid_tcp-center)@up)+2.5,up,motion['minimum_nominal_pad_height_cm'])
                        joint_margin=min(joint_margin,first['joint_limit_margin'])
                    else:
                        final_plan=plan_step(model,end,final-pre_tcp,pitch,tcp_offset,limits,
                            height_from_joints,up,motion['minimum_nominal_pad_height_cm'])
                except ValueError:
                    reason='悬停到低位抓点的受限逆解不可用'
                    rejections[reason]=rejections.get(reason,0)+1
                    continue
                joint_margin=min(joint_margin,final_plan['joint_limit_margin'])
            candidates.append((5*max(0,20-joint_margin)+2*abs(pitch-motion['preferred_nominal_pitch_deg']),solution['rounding_error_cm'],end,margin))
    return point,candidates


class PregraspPathUnavailable(ValueError):
    pass


PREFERRED_CLEARANCE_RESERVE_CM = .5


def build_pregrasp_plan(model,start,center,up,limits,tcp_offset,motion):
    preferred=motion['pregrasp_height_cm']
    heights=[preferred]
    adaptive=tcp_offset is not None and motion.get('adaptive_pitch_search')
    if adaptive:
        heights.extend(h for h in (preferred-.5,preferred+.5,preferred+1.)
                       if 4.2<=h<=8.5 and h-2.5>=motion['pregrasp_min_top_clearance_cm'])
    rejections={}
    plans=[]
    for height in heights:
        point,candidates=pregrasp_candidates(model,start,center,up,limits,tcp_offset,motion,height,rejections)
        plans.extend((row,point,height,False) for row in candidates)
    preferred_margin=motion['pregrasp_min_top_clearance_cm']+PREFERRED_CLEARANCE_RESERVE_CM
    if adaptive and motion.get('whole_path_precheck') and not any(row[3]>=preferred_margin for row,*_ in plans):
        # The safe and reachable band can lie between the coarse heights.
        ordered_heights=sorted(heights)
        midpoints=[(lo+hi)/2 for lo,hi in zip(ordered_heights,ordered_heights[1:])]
        split_heights=sorted(heights+midpoints,key=lambda height:(abs(height-preferred),height))
        lowest=min(heights)
        split_heights.extend(lowest+step for step in (.1,.15,.2) if lowest+step<=8.5)
        for height in split_heights:
            point,candidates=pregrasp_candidates(model,start,center,up,limits,tcp_offset,motion,height,rejections,split_descent=True)
            plans.extend((row,point,height,True) for row in candidates)
    if not plans:
        reasons='、'.join(f'{reason}（{count}）' for reason,count in rejections.items())
        raise PregraspPathUnavailable(f'悬停高度候选 {heights} cm 及适用的中间高度均无完整路径。{reasons or "悬停点没有可用逆解"}')
    def rank(plan):
        row,_,height,split=plan
        return (-min(row[3],preferred_margin),split,*row[:2],abs(height-preferred))
    selected,point,height,split_descent=min(plans,key=rank if adaptive else lambda plan:plan[0][:2])
    _,_,end,margin=selected
    count=math.ceil(max(abs(end[j]-start[j]) for j in ('3','4','5','6'))/motion['maximum_coarse_joint_delta'])
    waypoints=[{j:round(start[j]+(end[j]-start[j])*i/count) for j in ('3','4','5','6')} for i in range(1,count+1)]
    return {'point':point,'end':end,'margin':margin,'waypoints':waypoints,
            'split_descent_precheck':split_descent,
            'height_above_support_cm':height if tcp_offset is not None else 7.5}


def plan_box_pregrasp(model,start,center,up,limits,tcp_offset,motion,length_axis):
    try:
        planned=build_pregrasp_plan(model,start,center,up,limits,tcp_offset,motion)
        return {**planned,'grasp_offset_base_cm':[0.,0.,0.]}
    except PregraspPathUnavailable as center_error:
        along=motion['grasp_region']['along_length_cm']
        if tcp_offset is None or not motion.get('adaptive_pitch_search') or along<1.3:
            raise
        axis=np.array(length_axis,dtype=float)
        axis-=up*float(axis@up)
        axis_norm=np.linalg.norm(axis)
        if axis_norm<1e-8:
            raise
        axis/=axis_norm
        # Move along the long edge, toward the arm, keeping the closing
        # direction centered across the box and room inside the grasp band.
        radial_projection=float(axis[:2]@center[:2])
        if abs(radial_projection)<1e-8:
            raise
        inward=-axis if radial_projection>0 else axis
        failures=[]
        for distance in (.8,.4,1.2):
            if distance>along-.2:
                continue
            offset=distance*inward
            try:
                planned=build_pregrasp_plan(model,start,center+offset,up,limits,tcp_offset,motion)
            except PregraspPathUnavailable as offset_error:
                failures.append(f'{distance:g} cm：{offset_error}')
                continue
            return {**planned,'grasp_offset_base_cm':offset.tolist()}
        raise PregraspPathUnavailable(f'盒心路径无解：{center_error}；盒内偏移抓点亦无解：'+'；'.join(failures)) from center_error


def plan_clearance_recovery(model,start,center,up,limits,tcp_offset,motion,length_axis):
    minimum=motion['pregrasp_min_top_clearance_cm']
    gap=path_margin(start,start,model,center,up,limits,tcp_offset,0.)
    if not minimum-.25<=gap<minimum:
        return None
    pitch=model.forward({int(j):v for j,v in start.items()})['pitch_deg']
    for rise in (.5,.7):
        try:
            step=plan_step(model,start,rise*up,pitch,tcp_offset,limits,gap,up,gap)
            targets=step['targets']
            end={**start,**targets}
            path_margin(start,end,model,center,up,limits,tcp_offset,gap)
            if path_margin(end,end,model,center,up,limits,tcp_offset,minimum)<minimum+.25:
                continue
            plan_box_pregrasp(model,end,center,up,limits,tcp_offset,motion,length_axis)
        except ValueError:
            continue
        return targets
    return None


def split_pregrasp_segment(start,waypoint,model,center,up,limits,tcp_offset,minimum):
    end={**start,**waypoint}
    choices=[]
    for fraction in (.1,.2,.3,.4,.5,.6,.7,.8,.9):
        interpolated={j:round(start[j]+(value-start[j])*fraction) for j,value in waypoint.items()}
        for adjustment in (0,2,4,6):
            midpoint={**interpolated,'3':interpolated['3']+adjustment}
            if any(not limits[int(j)][0]<=value<=limits[int(j)][1] for j,value in midpoint.items()):
                continue
            if any(max(abs(a[j]-b[j]) for j in waypoint)<=10 for a,b in ((start,midpoint),(midpoint,end))):
                continue
            mid={**start,**midpoint}
            try:
                first=path_margin(start,mid,model,center,up,limits,tcp_offset,minimum)
                second=path_margin(mid,end,model,center,up,limits,tcp_offset,minimum)
            except PregraspClearanceError:
                continue
            choices.append((min(first,second),midpoint))
    return max(choices,key=lambda item:item[0])[1] if choices else None


def main(argv=None, camera=None, *, on_plan=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observation',type=Path)
    parser.add_argument('--arm-url',default='http://127.0.0.1:18000/v1/arm')
    parser.add_argument('--port',default='COM5')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--motion-profile',type=Path)
    parser.add_argument('--fallback-motion-profile',type=Path)
    parser.add_argument('--tcp-profile',type=Path,help='首次抓点候选，仅用于不接触的悬停验证')
    args=parser.parse_args(argv)
    motion=load_motion_profile(args.motion_profile)
    obs=json.loads(args.observation.read_text(encoding='utf-8'))
    geometry=obs['geometry'];target=geometry['selected']
    if target is None or not target['geometry_supported'] or obs['ocr']['target_selection']['status']!='target_found':
        raise ValueError('目标身份或几何不足')
    current=read_arm(args.arm_url);start=current['joint_median']
    if any(abs(start[j]-obs['after']['joint_median'][j])>8 for j in start):
        raise ValueError('当前姿态偏离定位图，需要重新观察')
    state=current['status']['data']
    if args.execute and state['capabilities'].get('writes_enabled') is not True:
        raise ValueError('运动未启用')
    limits={v['id']:(v['min'],v['max']) for v in state['servo_limits']}
    model=nominal_model();center=np.array(target['top_center_base_cm'])
    # Current 2.5cm box: first near TCP at 6cm above support, rather than
    # 6cm above its top (=8.5cm above support) followed by many tiny steps.
    up=-np.array(geometry['table_down_normal_base'])
    tcp_offset=None
    if args.tcp_profile:
        profile=json.loads(args.tcp_profile.read_text(encoding='utf-8'))
        if profile.get('allowed_stage')!='hover_only':raise ValueError('抓点配置不属于本阶段')
        require_start_state(start,profile['jaw_position'],profile['roll_position'],motion=motion)
        tcp_offset=np.array(profile['offset_tool_cm'],np.float64)
        if tcp_offset.shape!=(3,) or not np.isfinite(tcp_offset).all() or np.linalg.norm(tcp_offset)>4:
            raise ValueError('抓点候选无效')
    selection={'mode':'primary'}
    try:
        planned=plan_box_pregrasp(model,start,center,up,limits,tcp_offset,motion,target['length_axis_base'])
    except PregraspPathUnavailable as primary_error:
        if args.fallback_motion_profile is None:
            raise
        fallback=load_motion_profile(args.fallback_motion_profile)
        if args.tcp_profile:
            require_start_state(start,profile['jaw_position'],profile['roll_position'],motion=fallback)
        try:
            planned=plan_box_pregrasp(model,start,center,up,limits,tcp_offset,fallback,target['length_axis_base'])
        except PregraspPathUnavailable as fallback_error:
            raise PregraspPathUnavailable(f'常规和放宽俯仰均无解。常规：{primary_error}；放宽：{fallback_error}') from fallback_error
        motion=fallback
        args.motion_profile=args.fallback_motion_profile
        selection={'mode':'fallback','primary_error':str(primary_error)}
        print('常规俯仰无解，采用放宽后的俯仰范围：',motion['nominal_pitch_range_deg'],flush=True)
    point,end,margin,waypoints=(planned[k] for k in ('point','end','margin','waypoints'))
    count=len(waypoints)
    report={'type':'commissioning_pregrasp_clearance','source':str(args.observation.resolve()),
            'nominal_hover_cm':point.tolist(),'nominal_minimum_margin_cm':margin,
            'waypoints':waypoints,'commands':[],'completed':False,
            'physical_grasp_tcp_validated':False,'contact_motion_allowed':False}
    report['tcp_profile']=None if args.tcp_profile is None else str(args.tcp_profile.resolve())
    report['nominal_tcp_height_above_support_cm']=planned['height_above_support_cm']
    report['split_descent_precheck']=planned['split_descent_precheck']
    report['grasp_offset_base_cm']=planned['grasp_offset_base_cm']
    wrist_preview=preview_wrist_alignment(robot_frame(end,model,'pitch_tool')[:3,:3],target['width_axis_base'],up)
    report['wrist_alignment_preview']=wrist_preview
    report['motion_profile']=None if args.motion_profile is None else str(args.motion_profile.resolve())
    report['motion_profile_selection']=selection
    if on_plan is not None:
        on_plan(report['motion_profile'],motion,selection)
    print('预接近方案：',waypoints,'模型最小余量cm：',round(margin,2),
          '悬停距支撑面cm：',planned['height_above_support_cm'],flush=True)
    if planned['split_descent_precheck']:
        print('低位抓点通过两段名义下降检查，实机仍按视觉反馈逐步接近。',flush=True)
    if any(planned['grasp_offset_base_cm']):
        print(f"采用盒内沿长边偏移 {np.linalg.norm(planned['grasp_offset_base_cm']):.1f} cm 的抓点。",flush=True)
    if wrist_preview['status']=='nominal_roll_preview':
        print('悬停腕轴预览：名义相对转角',round(wrist_preview['relative_roll_deg'],1),
              '度，尚无二号舵机标定。',flush=True)
    if not args.execute:return
    output=ROOT/'vision_grasp_results'/('hover_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.json')
    def save():output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    try:
        i=0
        while i<len(waypoints):
            waypoint=waypoints[i]
            i+=1
            before=current if i==1 else coarse_feedback(args.arm_url)
            if 'open_jaw_minimum' in motion and tcp_offset is not None:
                if i>1 and start_state_error(before['joint_median'],profile['jaw_position'],profile['roll_position'],motion=motion):
                    before=read_arm(args.arm_url)
                require_start_state(before['joint_median'],profile['jaw_position'],profile['roll_position'],motion=motion)
            elif any(abs(before['joint_median'][j]-start[j])>3 for j in ('1','2')):
                raise RuntimeError('夹爪或腕轴状态变化')
            limits={v['id']:(v['min'],v['max']) for v in before['status']['data']['servo_limits']}
            measured=before['joint_median']
            clearance=motion['pregrasp_min_top_clearance_cm']
            if report.get('clearance_recovery',{}).get('before_segment')==i-1:
                gap=path_margin(measured,measured,model,center,up,limits,tcp_offset,0.)
                report['clearance_recovery']['feedback_gap_cm']=gap
                save()
                if gap<clearance:
                    raise PregraspClearanceError(f'抬高后实读净空仍不足（{gap:.3f} cm，要求 {clearance:.3f} cm）')
            if (tcp_offset is not None and motion.get('adaptive_pitch_search')
                    and 'clearance_recovery' not in report):
                recovery=plan_clearance_recovery(model,measured,center,up,limits,tcp_offset,motion,
                                                 target['length_axis_base'])
                if recovery is not None:
                    report['clearance_recovery']={'before_segment':i,'joints':measured,'targets':recovery}
                    waypoints=waypoints[:i-1]+[recovery]+waypoints[i-1:]
                    report['waypoints']=waypoints
                    count=len(waypoints)
                    waypoint=recovery
                    clearance=path_margin(measured,measured,model,center,up,limits,tcp_offset,0.)
                    save()
                    print('当前悬停余量略低，先抬高夹爪，再按实读检查下一段。',flush=True)
            try:
                path_margin(measured,{**measured,**waypoint},model,center,up,limits,tcp_offset,clearance)
            except PregraspClearanceError as error:
                if motion.get('adaptive_pitch_search') and 'feedback_segment_split' not in report:
                    midpoint=split_pregrasp_segment(measured,waypoint,model,center,up,limits,tcp_offset,clearance)
                    if midpoint is not None:
                        report['feedback_segment_split']={'before_segment':i,'joints':measured,
                                                          'midpoint':midpoint,'reason':str(error)}
                        waypoints=waypoints[:i-1]+[midpoint]+waypoints[i-1:]
                        report['waypoints']=waypoints
                        count=len(waypoints)
                        save()
                        print('直达悬停的路径余量不足，已检查通过两段路径。',flush=True)
                        i-=1
                        continue
                if 'feedback_replan' in report or not motion.get('adaptive_pitch_search'):
                    raise
                report['feedback_replan']={'before_segment':i,'joints':measured,'reason':str(error)}
                save()
                revised=plan_box_pregrasp(model,measured,center,up,limits,tcp_offset,motion,target['length_axis_base'])
                suffix=revised['waypoints']
                segment_start=measured
                for candidate in suffix:
                    segment_end={**segment_start,**candidate}
                    path_margin(segment_start,segment_end,model,center,up,limits,tcp_offset,clearance)
                    segment_start=segment_end
                waypoints=waypoints[:i-1]+suffix
                count=len(waypoints)
                report.update(waypoints=waypoints,nominal_hover_cm=revised['point'].tolist(),
                              nominal_minimum_margin_cm=revised['margin'],
                              nominal_tcp_height_above_support_cm=revised['height_above_support_cm'],
                              split_descent_precheck=revised['split_descent_precheck'],
                              grasp_offset_base_cm=revised['grasp_offset_base_cm'],
                              wrist_alignment_preview=preview_wrist_alignment(
                                  robot_frame(revised['end'],model,'pitch_tool')[:3,:3],target['width_axis_base'],up))
                save()
                print('实际关节反馈改变了剩余路径，已重新通过预接近检查。',flush=True)
                if not suffix:
                    break
                waypoint=suffix[0]
            body={'request_id':'hover_'+uuid.uuid4().hex,'targets':[{'id':int(j),'position':v} for j,v in waypoint.items()],'duration_ms':20}
            command={'request':body,'response':None,'task':None};report['commands'].append(command);save()
            result=call(args.arm_url,'move',body);command['response']=result;save()
            task=result['task'];deadline=time.monotonic()+22
            while task['status']=='sent' and time.monotonic()<deadline:
                time.sleep(.3);task=call(args.arm_url,'tasks/'+task['task_id'])
            command['task']=task;save()
            if task['status']!='reached':raise RuntimeError('运动未确认：'+str(task))
            print('预接近',i,'/',count,'已到位',flush=True)
        time.sleep(.8)
        captures=[]
        for attempt in range(2):
            before=read_arm(args.arm_url)
            if camera is None:
                image=receive_photos(args.port,115200,ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d'),once=True,request_capture=True)
            else:
                capture=camera.capture('hover_'+uuid.uuid4().hex)
                if capture['status']!='captured':raise RuntimeError('预接近拍照失败：'+str(capture))
                image=Path(capture['image_path'])
            if image is None:raise RuntimeError('未取得近图')
            after=read_arm(args.arm_url);integrity=check_jpeg(image)
            stable=(all(abs(before['joint_median'][j]-after['joint_median'][j])<=2 for j in before['joint_median'])
                    and all(v<=6 for reading in (before,after) for v in reading['joint_span'].values())
                    and all(v<=2 for reading in (before,after) for v in reading['joint_mad'].values()))
            captures.append({'image':str(image),'before':before,'after':after,'integrity':integrity,
                             'joint_feedback_stable':stable})
            report['captures']=captures;save()
            if integrity['valid'] and stable:break
            if attempt==0:
                print('近图校验或关节反馈未通过，原地等候后重拍一次。',flush=True)
                time.sleep(.8)
        else:raise RuntimeError('两次近图均未通过图像完整性和关节稳定检查')
        reference=cv2.imdecode(np.frombuffer(Path(obs['image']).read_bytes(),np.uint8),0)
        frame=cv2.imdecode(np.frombuffer(Path(image).read_bytes(),np.uint8),0)
        cal=json.loads(Path(obs['calibration']).read_text(encoding='utf-8'))['candidates'][0]
        camera=robot_frame(after['joint_median'],model,'pitch_tool')@np.array(cal['T_mount_camera_cm'])
        inv=np.linalg.inv(camera)
        intrinsic=json.loads((ROOT/'camera_calibration_results'/'diagnostic_20261026_58dc9618'/'intrinsics_candidate.json').read_text(encoding='utf-8'))
        corners=np.array(target['top_corners_base_cm'])
        predicted=cv2.projectPoints(corners,cv2.Rodrigues(inv[:3,:3])[0],inv[:3,3],np.array(intrinsic['camera_matrix_px']),np.array(intrinsic['distortion_coefficients']))[0].reshape(-1,2)
        expected_scale=abs(cv2.contourArea(predicted.astype(np.float32)))/abs(cv2.contourArea(np.array(target['corners_pixel'],np.float32)))
        tracked=track(reference,frame,target['corners_pixel'],scale_bounds=(expected_scale*.6,expected_scale*1.6),coverage_basis='common_visible',current_face_hint=predicted)
        report.update(completed=True,hover_motion_completed=True,
                      near_tracking_accepted=tracked['status']=='tracked_face_for_review',
                      final=after,tracking=tracked,predicted_corners_pixel=predicted.tolist(),expected_scale=expected_scale)
        print('近端跟踪：',tracked,flush=True)
    except BaseException as error:
        report['error']=str(error);raise
    finally:save();print('预接近记录：',output,flush=True)


if __name__=='__main__':main()
