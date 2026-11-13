"""Commissioning visual approach to the box; no closing, lift or recovery.

The coarse two-view TCP is an estimate, not a certified calibration. Every step
requires independent fixed-marker camera pose and tracked stationary box evidence.
"""
import argparse
from contextlib import nullcontext
from datetime import datetime
import json
from pathlib import Path
import time
from types import SimpleNamespace
import uuid
import numpy as np
from check_grasp_board import ROOT
from collect_grasp_reference import read_arm
from grasp_visual_observer import GraspVisualObserver
from jpeg_integrity import check_jpeg
from sample_grasp_handeye import call
from vision_task import VisionTaskSession
from tcp_pose_planner import choose_step
from sample_grasp_handeye import nominal_model
from grasp_start_state import require_start_state
from grasp_approach_policy import approach_decision,grasp_point_board,GRASP_REGION
from grasp_motion_profile import load_motion_profile,pitch_in_profile


def compensated_targets(goal,previous,actual,limits):
    if any(int(j) not in (3,4,5,6) for j in goal):raise ValueError('补偿只用于3..6号')
    errors={j:goal[j]-actual[j] for j in goal}
    if max(abs(v) for v in errors.values())>20:raise RuntimeError('实际反馈远离本步关节目标')
    result={}
    for j in goal:
        change=max(-5,min(5,errors[j]))
        value=previous[j]+change
        value=max(goal[j]-10,min(goal[j]+10,value))
        lo,hi=limits[int(j)]
        result[j]=max(lo,min(hi,value))
    return result


def main(argv=None, camera_session=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm-url',default='http://127.0.0.1:18000/v1/arm')
    p.add_argument('--port',default='COM7');p.add_argument('--execute',action='store_true')
    p.add_argument('--max-steps',type=int)
    p.add_argument('--motion-profile',type=Path)
    p.add_argument('--motion-backend',choices=('coordinate','joint_preview','tcp_pose_ik'),default='tcp_pose_ik')
    p.add_argument('--observation',type=Path)
    p.add_argument('--hover-report',type=Path,help='同一场景预接近的近图，避免阶段切换丢失近景关键帧')
    p.add_argument('--seed-approach',type=Path,help='只加载本轮已验收图像，不重发旧运动')
    a=p.parse_args(argv)
    motion=load_motion_profile(a.motion_profile)
    a.max_steps=motion['maximum_near_corrections'] if a.max_steps is None else min(a.max_steps,motion['maximum_near_corrections'])
    if not 1<=a.max_steps<=10:raise ValueError('本次试验最多10步')
    source=a.observation or ROOT/'vision_grasp_results/photo_20261105_120734_974932_seq1_396c5888_d558ca46_commissioning_boardanchor.json'
    profile=ROOT/'calibration_boards/grasp_tcp_hover_candidate_20261105.json'
    observer=GraspVisualObserver(source,profile)
    initial_center=(np.linalg.inv(observer.Y)@np.r_[
        observer.observation['geometry']['selected']['top_center_base_cm'],1.])[:2]
    output=ROOT/'vision_grasp_results'/('visual_approach_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.json')
    report={'type':'commissioning_guarded_visual_approach','phase':'starting','completed':False,
        'production_ready':False,'tcp_estimate_not_accuracy_certificate':True,
        'source':str(source),'tcp_profile':str(profile),'observations':[],'commands':[],
        'gripper_close_sent':False,'max_steps':a.max_steps}
    report['grasp_region']=motion['grasp_region']
    report['motion_profile']=None if a.motion_profile is None else str(a.motion_profile.resolve())
    cached=None
    resumed_last_move=None
    grasp_offset_board=None
    def save():output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    def check_pose(r):
        require_start_state(r['joint_median'],motion=motion)
        if any(v>6 for v in r['joint_span'].values()) or any(v>2 for v in r['joint_mad'].values()):
            raise RuntimeError('当前反馈不稳定')
    runner=SimpleNamespace(target={'id':'med_003','name':'阿昔洛韦乳膏'})
    try:
        if a.hover_report:
            hover=json.loads(a.hover_report.read_text(encoding='utf-8'))
            if hover.get('grasp_offset_base_cm'):
                grasp_offset_board=observer.Y[:3,:3].T@np.array(hover['grasp_offset_base_cm'])
            if Path(hover['source']).resolve()!=source.resolve() or not hover.get('hover_motion_completed'):
                raise ValueError('近景关键帧不属于本轮已完成的预接近')
            if grasp_offset_board is not None:
                report['grasp_offset_board_cm']=grasp_offset_board.tolist()
            packet=hover['captures'][-1]
            report['hover_handoff']={'report':str(a.hover_report.resolve())}
            try:
                bootstrap=observer.estimate(Path(packet['image']),packet['after']['joint_median'])
            except ValueError as error:
                bootstrap=None
                report['hover_handoff']['visual_error']=str(error)
                print('悬停近图定位失败，将在原地重新拍照。',flush=True)
            else:
                report['hover_handoff']['visual']=bootstrap
            save()
            if bootstrap is not None and not a.seed_approach and time.time()-datetime.fromisoformat(packet['after']['received_at']).timestamp()<=motion['reuse_fresh_stationary_frame_s']:
                cached={'capture':{'image_path':packet['image'],'reused_from_hover':str(a.hover_report.resolve())},
                        'before':packet['before'],'after':packet['after'],'visual':bootstrap,'integrity':packet['integrity']}
        if a.seed_approach:
            seed=json.loads(a.seed_approach.read_text(encoding='utf-8'))
            if Path(seed['source']).resolve()!=source.resolve() or seed.get('gripper_close_sent'):
                raise ValueError('视觉接续记录不属于同一个未闭爪场景')
            if any(c.get('task',{}).get('status')!='reached' for c in seed.get('commands',[])):
                raise ValueError('接续记录存在未知或未完成运动，不自动恢复')
            observer.restore_grounded_journal(seed)
            if seed.get('commands'):
                resumed_last_move=seed['commands'][-1]
            report['seed_approach']=str(a.seed_approach.resolve());save()
        session_context = nullcontext(camera_session) if camera_session is not None else VisionTaskSession(
            a.port,'med_003',images=ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d'),runner=runner)
        with session_context as camera:
            def observe():
                errors=[]
                for attempt in range(2):
                    for settle in range(3):
                        before=read_arm(a.arm_url)
                        report.setdefault('pre_capture_reads',[]).append(before);save()
                        try:check_pose(before);break
                        except RuntimeError:
                            if settle==2:raise
                            time.sleep(.5)
                    c=camera.capture('guarded_obs_'+uuid.uuid4().hex)
                    if c['status']!='captured':raise RuntimeError('采传失败：'+str(c))
                    after=read_arm(a.arm_url);check_pose(after)
                    if any(abs(before['joint_median'][j]-after['joint_median'][j])>2 for j in before['joint_median']):
                        raise RuntimeError('拍照期间关节反馈改变')
                    path=Path(c['image_path']);integrity=check_jpeg(path)
                    packet={'capture':c,'before':before,'after':after,'integrity':integrity}
                    report['observations'].append(packet);save()
                    if not integrity['valid']:
                        errors.append('近图JPEG校验失败：'+str(integrity))
                        if attempt==0:
                            print('近图文件未通过JPEG解码检查，原地重拍一次。',flush=True)
                        continue
                    try:
                        visual=observer.estimate(path,after['joint_median'])
                    except ValueError as error:
                        errors.append(str(error))
                        packet['visual_error']=str(error);save()
                        if attempt==0:print('近图定位失败，原地重拍一次。',flush=True)
                        continue
                    packet['visual']=visual;save()
                    return visual,after
                raise ValueError('两次近图均未通过观测：'+' | '.join(errors))
            for iteration in range(a.max_steps+1):
                if iteration==0 and cached is not None:
                    state=read_arm(a.arm_url);check_pose(state)
                    if (time.time()-datetime.fromisoformat(cached['after']['received_at']).timestamp()<=motion['reuse_fresh_stationary_frame_s']
                        and all(abs(state['joint_median'][j]-cached['after']['joint_median'][j])<=2 for j in state['joint_median'])):
                        cached['reuse_validation_read']=state;report['observations'].append(cached);save();visual=cached['visual']
                        print('复用已验收的悬停近图。',flush=True)
                    else:visual,state=observe()
                else:visual,state=observe()
                tcp=np.array(visual['tcp_board_cm'])
                obj=np.array(visual['T_board_object'])
                goal=grasp_point_board(obj,grasp_offset_board)
                goal[2]=-motion['grasp_region']['tcp_height_target_cm']
                error=goal-tcp
                height=-tcp[2]
                last_command=report['commands'][-1] if report['commands'] else resumed_last_move
                if last_command and 'planned_rise_cm' in last_command:
                    actual_rise=height-last_command['start_tcp_height_cm']
                    report['last_rise_check']={'planned_cm':last_command['planned_rise_cm'],
                                               'observed_cm':float(actual_rise)}
                    save()
                    if actual_rise<=0:
                        raise RuntimeError(f"抬高后近图高度未增加：计划上升 {last_command['planned_rise_cm']:.2f} cm，实测变化 {actual_rise:+.2f} cm")
                if motion.get('adaptive_pitch_search'):
                    shift=float(np.linalg.norm(obj[:2,3]-initial_center))
                    report['box_center_shift_from_initial_cm']=shift;save()
                    if shift>.6:
                        raise RuntimeError(f'本轮盒心与初图相差 {shift:.2f} cm，请核对现场后重新从观察位拍照')
                    if iteration==0 and not a.seed_approach and height<4.2:
                        raise RuntimeError(f'首次近景实测留空不足，胶垫代理高度 {height:.2f} cm')
                decision=approach_decision(tcp,obj,motion,grasp_offset_board=grasp_offset_board)
                pitch=nominal_model().forward({int(j):v for j,v in state['joint_median'].items()})['pitch_deg']
                pose_ready=pitch_in_profile(pitch,motion)
                print('视觉步',iteration,'TCP距支撑面cm：',round(height,2),'目标误差板坐标cm：',np.round(error,2).tolist(),flush=True)
                if not motion['minimum_nominal_pad_height_cm']<=height<=12:raise RuntimeError('TCP高度超出本次试验范围')
                if decision['ready'] and pose_ready:
                    report.update(completed=True,phase='ready_for_separate_closure_check',final_observation=visual)
                    break
                if iteration==a.max_steps:raise RuntimeError(f'达到{a.max_steps}步预算，未满足接近门槛')
                report['approach_decision']=decision;save()
                delta=observer.Y[:3,:3]@np.array(decision['delta_board_cm'])
                delta=np.round(delta,1)
                if np.linalg.norm(delta)>motion['maximum_near_step_cm']+.05 or (np.linalg.norm(delta)<.15 and pose_ready):
                    raise RuntimeError('本步不满足有限位移门槛')
                preview=None
                if a.motion_backend=='tcp_pose_ik':
                    model=nominal_model();positions=state['joint_median']
                    limits={v['id']:(v['min'],v['max']) for v in state['status']['data']['servo_limits']}
                    joint_bias=None
                    previous_joint_delta=None
                    last_move=report['commands'][-1] if report['commands'] else resumed_last_move
                    if (motion.get('adaptive_pitch_search') and last_move is not None
                            and last_move.get('endpoint')=='move' and last_move.get('start_joint_positions')):
                        last_targets=last_move['request']['targets']
                        measured={str(v['id']):positions[str(v['id'])]-v['position'] for v in last_targets}
                        if all(abs(value)<=10 for value in measured.values()):
                            joint_bias=measured
                            previous_joint_delta={str(v['id']):v['position']-last_move['start_joint_positions'][str(v['id'])]
                                                  for v in last_targets}
                    minimum_height=motion['minimum_nominal_pad_height_cm']
                    if decision['phase']=='horizontal_alignment_at_clearance':
                        minimum_height=max(minimum_height,3.0)
                    planned=choose_step(model,positions,delta,observer.tcp['offset_tool_cm'],limits,height,-observer.Y[:3,2],
                        minimum_height,motion['preferred_nominal_pitch_deg'],motion['nominal_pitch_range_deg'],
                        joint_bias=joint_bias,previous_joint_delta=previous_joint_delta)
                    preview={'status':'client_preview','coordinate_plan':planned}
                    delta=np.array(planned['predicted_delta_cm'])
                for scale in (1.,.5):
                    if preview is not None:break
                    proposal=np.round(delta*scale,1)
                    body={'dx_cm':float(proposal[0]),'dy_cm':float(proposal[1]),'dz_cm':float(proposal[2])}
                    try:
                        candidate=call(a.arm_url,'coordinate/preview',{'request_id':'guarded_preview_'+uuid.uuid4().hex,**body})
                    except RuntimeError as error:
                        message=str(error)
                        details=json.loads(message[message.find('{'):]) if '{' in message else {}
                        e=details.get('error',{})
                        if e.get('outcome_unknown') is False and e.get('code') in ('coordinate_joint_step_too_large','coordinate_unreachable','coordinate_rounding_error'):
                            report.setdefault('rejected_previews',[]).append({'body':body,'error':details});save();continue
                        raise
                    if candidate['coordinate_plan'].get('feedback_detectable'):
                        preview=candidate;delta=proposal;break
                if preview is None:raise RuntimeError('有限候选步长均无可反馈的可达方案')
                plan=preview['coordinate_plan']
                report['next_preview']=preview;save()
                if not plan.get('feedback_detectable') or plan['max_joint_delta']>100:
                    raise RuntimeError('预览不满足反馈分辨率/关节步幅')
                predicted=np.array(plan['predicted_delta_cm'])
                if np.linalg.norm(predicted-delta)>.15:raise RuntimeError('预览取整误差过大')
                if not a.execute:
                    report['phase']='preview_only';break
                if a.motion_backend in ('joint_preview','tcp_pose_ik'):
                    request={'request_id':'guarded_joint_'+uuid.uuid4().hex,
                        'targets':[{'id':int(j),'position':int(v)} for j,v in plan['targets'].items()],
                        'duration_ms':20}
                    if a.motion_backend=='tcp_pose_ik' and plan.get('observed_joint_bias_used'):
                        # The service compares to the bias-adjusted command.
                        # Keep the original ten-unit check against the IK goal.
                        request['position_tolerance']=12
                    endpoint='move'
                else:
                    request={'request_id':'guarded_step_'+uuid.uuid4().hex,**body};endpoint='coordinate/increment'
                command={'request':request,'endpoint':endpoint,'preview':preview,
                    'acceptance_note':'Joint feedback only; subsequent external visual observation decides actual approach progress.'}
                if decision['delta_board_cm'][2]<0:
                    planned_rise=float(predicted@(-observer.Y[:3,2]))
                    if planned_rise<=0:
                        raise RuntimeError('抬高请求未生成向上的预览动作')
                    command.update(planned_rise_cm=planned_rise,start_tcp_height_cm=float(height))
                if a.motion_backend=='tcp_pose_ik':
                    command['start_joint_positions']=positions.copy()
                report['commands'].append(command);save()
                result=call(a.arm_url,endpoint,request);command['response']=result;save()
                task=result['task'];deadline=time.monotonic()+26
                while task['status']=='sent' and time.monotonic()<deadline:
                    time.sleep(.3);task=call(a.arm_url,'tasks/'+task['task_id'])
                command['task']=task;save()
                if task['status']!='reached':raise RuntimeError('动作未确认到位：'+str(task))
                if request.get('position_tolerance')==12:
                    actual=task.get('position_feedback',{}).get('joint_positions',{})
                    expected=plan['expected_joint_positions']
                    if any(j not in actual or abs(actual[j]-v)>10 for j,v in expected.items()):
                        raise RuntimeError('补偿动作已返回反馈，但关节位置偏离原规划目标超过10单位')
                if a.motion_backend in ('joint_preview','tcp_pose_ik') and not motion.get('adaptive_pitch_search'):
                    goal={str(j):int(v) for j,v in plan['targets'].items()}
                    previous={str(v['id']):v['position'] for v in request['targets']}
                    command['intended_joint_goal']=goal
                    for correction in range(2):
                        actual=read_arm(a.arm_url);positions=actual['joint_median']
                        if max(abs(goal[j]-positions[j]) for j in goal)<=3:break
                        limits={v['id']:(v['min'],v['max']) for v in actual['status']['data']['servo_limits']}
                        adjusted=compensated_targets(goal,previous,positions,limits)
                        if adjusted==previous:break
                        request2={'request_id':'joint_observed_comp_'+uuid.uuid4().hex,
                                  'targets':[{'id':int(j),'position':v} for j,v in adjusted.items()], 'duration_ms':20}
                        entry={'request':request2,'before':actual,'purpose':'bounded feedback compensation, not a move replay'}
                        command.setdefault('joint_feedback_corrections',[]).append(entry);save()
                        result2=call(a.arm_url,'move',request2);entry['response']=result2;save()
                        task2=result2['task'];deadline=time.monotonic()+22
                        while task2['status']=='sent' and time.monotonic()<deadline:
                            time.sleep(.3);task2=call(a.arm_url,'tasks/'+task2['task_id'])
                        entry['task']=task2;save()
                        if task2['status']!='reached':raise RuntimeError('关节补偿结果未知，结束本轮')
                        previous=adjusted
                time.sleep(.8)
            else:raise RuntimeError('接近循环异常结束')
    except BaseException as error:
        report.update(phase='failed',failure_reason=str(error));raise
    finally:save();print('本轮记录：',output,flush=True)


if __name__=='__main__':main()
