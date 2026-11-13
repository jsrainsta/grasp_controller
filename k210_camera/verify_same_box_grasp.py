"""Explicit current-box commissioning close/lift trial, never a force certificate.

Requires a completed visual approach journal and a fresh independent recheck.
The generic MOVE endpoint completes only coarse position observation; the host
separately checks stable jaw feedback and free object pose during a lift.
Allows one bounded lift compensation from settled joint feedback.
"""
import argparse,json,time,uuid
from contextlib import ExitStack,nullcontext
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from check_grasp_board import ROOT
from collect_grasp_reference import read_arm
from sample_grasp_handeye import call,nominal_model
from grasp_visual_observer import GraspVisualObserver,discrepancy
from vision_task import VisionTaskSession
from jpeg_integrity import check_jpeg
from tcp_pose_planner import plan_step,plan_verification_lift
from grasp_start_state import require_start_state
from grasp_approach_policy import in_grasp_region,GRASP_REGION
from grasp_motion_profile import load_motion_profile,pitch_in_profile
from fit_grasp_handeye import robot_frame,preview_wrist_alignment
from register_current_scene import undirected_angle_deg

def stable_loaded(samples):
    # Same physical box only. This is not contact/force/grasp detection.
    return len(samples)>=4 and all(480<=v<=505 for v in samples) and max(samples)-min(samples)<=3


def can_reuse_lift_baseline(before, after, visual):
    positioning_joints=('2','3','4','5','6')
    return (visual.get('object_pose_basis')=='known_stationary_support_plane'
            and abs(visual['minimum_box_bottom_height_cm'])<=.7
            and all(abs(before['joint_median'][j]-after['joint_median'][j])<=2
                    and after['joint_span'][j]<=6 and after['joint_mad'][j]<=2
                    for j in positioning_joints))


def stable_hold_feedback(lifted,held):
    first,second=lifted['joint_median'],held['joint_median']
    return (480<=first['1']<=505 and 480<=second['1']<=505
            and all(abs(first[j]-second[j])<=3 for j in first)
            and all(span<=6 for span in held['joint_span'].values())
            and all(mad<=2 for mad in held['joint_mad'].values()))

def bottom_lift_metrics(baseline, observation):
    before=baseline['minimum_box_bottom_height_cm']
    after=observation['minimum_box_bottom_height_cm']
    return {'before_cm':before,'after_cm':after,'rise_cm':after-before,
            'passed':after>0 and after-before>.7}

def held_recheck_reference(trial):
    commands=trial['commands']
    if (trial.get('grasp_confirmed') or len(commands) not in (2,3)
            or (len(commands)==3 and commands[-1].get('purpose')!='试提关节反馈补偿')
            or any(c.get('task',{}).get('status')!='reached' for c in commands)
            or not stable_loaded(trial.get('jaw_samples',[]))):
        raise ValueError('此记录缺少已确认的闭爪和试提，不能只补拍验收')
    reference=commands[-1]['task'].get('position_feedback',{}).get('joint_positions',{})
    if set(reference)!=set('123456') or not 480<=reference['1']<=505:
        raise ValueError('此记录缺少试提后的持物关节反馈')
    baseline_index=0 if trial.get('lift_baseline_source')=='preclosure_accepted_support_observation' else 1
    baseline=trial['observations'][baseline_index]['visual']
    if not -.7<=baseline['minimum_box_bottom_height_cm']<=.7:
        raise ValueError('缺少试提前的有效支撑面基准')
    return reference,baseline

def main(argv=None, camera_session=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('approach',type=Path);p.add_argument('--port',required=True,help='实际 COM 号，补验时也可用 auto')
    p.add_argument('--arm-url',default='http://127.0.0.1:18000/v1/arm')
    p.add_argument('--camera-baudrate',type=int,default=115200)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--recheck-held',action='store_true',help='输入已试提记录，只补拍和读反馈，不发送动作')
    a=p.parse_args(argv)
    prior=json.loads(a.approach.read_text(encoding='utf-8')) if a.recheck_held else None
    approach_path=Path(prior['source']) if prior is not None else a.approach
    journal=json.loads(approach_path.read_text(encoding='utf-8'))
    motion=load_motion_profile(journal.get('motion_profile'))
    if not journal.get('completed') or journal.get('phase')!='ready_for_separate_closure_check':
        raise ValueError('没有成功完成的视觉接近记录')
    if time.time()-a.approach.stat().st_mtime>(3600 if a.recheck_held else 600):
        raise ValueError('现场接续记录已过期')
    if a.recheck_held:
        reference,baseline=held_recheck_reference(prior)
    observer=GraspVisualObserver(Path(journal['source']),Path(journal['tcp_profile']))
    observer.restore_grounded_journal(journal)
    output=ROOT/'vision_grasp_results'/('grasp_trial_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.json')
    result={'type':'same_box_commissioning_grasp_trial','source':str(a.approach.resolve()),
        'production_ready':False,'grasp_confirmed':False,'force_measured':False,
        'grasp_quality_validated':False,
        'jaw_target':520,'coarse_position_tolerance':40,'loaded_observation_band':[480,505],
        'commands':[],'observations':[],'phase':'starting'}
    if prior is not None:
        result.update(prior)
        result.pop('error',None)
        result.update(rechecked_from=str(a.approach.resolve()),hardware_motion_sent=False,phase='holding_not_verified')
    def save():output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    def move(targets,purpose,tolerance=10):
        read_arm(a.arm_url)
        request={'request_id':'grasp_trial_'+uuid.uuid4().hex,'targets':[{'id':int(j),'position':int(v)} for j,v in targets.items()],
                 'duration_ms':20,'position_tolerance':tolerance}
        c={'purpose':purpose,'request':request};result['commands'].append(c);save()
        r=call(a.arm_url,'move',request);c['response']=r;save();task=r['task'];deadline=time.monotonic()+24
        while task['status']=='sent' and time.monotonic()<deadline:
            time.sleep(.25);task=call(a.arm_url,'tasks/'+task['task_id'])
        c['task']=task;save()
        if task['status']!='reached':raise RuntimeError('动作结果未知，停止后续步骤：'+str(task))
    try:
        with ExitStack() as stack:
            if a.recheck_held:
                from grasp_controller import ControllerLock,resolve_port
                stack.enter_context(ControllerLock(ROOT/'controller_results/controller.lock'))
                a.port=resolve_port(a.port)
            session_context = nullcontext(camera_session) if camera_session is not None else VisionTaskSession(
                a.port,'med_003',images=ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d'),
                baudrate=a.camera_baudrate,
                runner=SimpleNamespace(target={'id':'med_003','name':'阿昔洛韦乳膏'}))
            camera=stack.enter_context(session_context)
            def observe(held,stationary_reference=None):
                last_error=None
                for attempt in range(2):
                    before=read_arm(a.arm_url);r=camera.capture('grasp_verify_'+uuid.uuid4().hex)
                    if r.get('status')!='captured':raise RuntimeError('拍照失败：'+str(r))
                    after=read_arm(a.arm_url);path=Path(r['image_path'])
                    integrity=check_jpeg(path)
                    if not integrity['valid']:
                        last_error=ValueError('试提照片JPEG校验失败：'+str(path))
                        result.setdefault('observation_rejections',[]).append(
                            {'image':str(path),'jpeg_integrity':integrity});save()
                        if attempt==0:
                            print('验收图未通过JPEG解码检查，原地重拍一次。',flush=True)
                        continue
                    if any(abs(before['joint_median'][j]-after['joint_median'][j])>3 for j in before['joint_median']):
                        raise RuntimeError('曝光期间关节反馈改变')
                    packet={'before':before,'after':after,'image':str(path)}
                    try:
                        v=observer.estimate(path,after['joint_median'],held=held,stationary_reference=stationary_reference)
                    except ValueError as error:
                        last_error=error
                        packet['visual_error']=str(error)
                        result.setdefault('observation_rejections',[]).append(packet);save()
                        if attempt==0:print('验收图定位失败，原地重拍一次。',flush=True)
                        continue
                    packet['visual']=v
                    result['observations'].append(packet);save()
                    return v,after
                raise ValueError('两次验收图均未通过观测：'+str(last_error)) from last_error
            def verify_lift(baseline):
                lifted,lift_state=observe(True)
                lift_bottom=bottom_lift_metrics(baseline,lifted)
                result['visual_lift_bottom']=lift_bottom
                time.sleep(3.)
                if motion.get('verify_hold_with_joint_feedback'):
                    state=read_arm(a.arm_url)
                    rise=lifted['top_height_cm']-baseline['top_height_cm']
                    angle=None
                    checks={
                        'lift_bottom_clearance':lift_bottom['passed'],
                        'positive_object_rise':rise>0,
                        'stable_loaded_joint_feedback':stable_hold_feedback(lift_state,state),
                    }
                    result.update(verification_method='visual_lift_and_stable_joint_feedback',hold_feedback=state)
                else:
                    held,state=observe(True,stationary_reference=(lifted,lift_state['joint_median']))
                    angle=discrepancy(np.array(lifted['T_camera_object']),np.array(held['T_camera_object']))
                    rise=held['top_height_cm']-baseline['top_height_cm']
                    held_bottom=bottom_lift_metrics(baseline,held)
                    result['visual_held_bottom']=held_bottom
                    checks={
                        'lift_bottom_clearance':lift_bottom['passed'],
                        'held_bottom_clearance':held_bottom['passed'],
                        'object_rise':rise>=1.2,
                        'relative_pose_stability':angle[0]<=.4 and angle[1]<=5,
                    }
                passed=all(checks.values())
                result.update(grasp_confirmed=bool(passed),phase='held_verified' if passed else 'holding_not_verified',
                              object_rise_cm=rise,relative_pose_drift_cm_deg=None if angle is None else list(angle),
                              verification_checks=checks)
                print(f"视觉盒顶升高 {rise:.2f} cm，最低角试提前 {lift_bottom['before_cm']:.2f} cm，"
                      f"试提后 {lift_bottom['after_cm']:.2f} cm，相对升高 {lift_bottom['rise_cm']:.2f} cm。",flush=True)
                if not passed:
                    labels={'lift_bottom_clearance':'试提后最低角离面或相对升高不足',
                            'held_bottom_clearance':'保持时最低角离面或相对升高不足',
                            'positive_object_rise':'盒顶未升高','object_rise':'盒顶升高不足',
                            'stable_loaded_joint_feedback':'持物关节反馈不稳定',
                            'relative_pose_stability':'药盒相对相机位姿变化过大'}
                    failed='、'.join(f'{labels[name]}（{name}）' for name,ok in checks.items() if not ok)
                    raise RuntimeError('试提验收未通过：'+failed)
                print('本盒试提通过：照片确认离面，保持期间舵机反馈稳定。' if angle is None
                      else '本盒试提通过：物体离面并保持稳定。',flush=True)

            if a.recheck_held:
                state=read_arm(a.arm_url)
                if not stable_hold_feedback({'joint_median':reference},state):
                    raise RuntimeError('当前关节反馈与试提结束位置不符，无法补验')
                verify_lift(baseline)
                return
            cached=journal['observations'][-1]
            age=time.time()-datetime.fromisoformat(cached['after']['received_at']).timestamp()
            reused=False
            if age<=motion['reuse_fresh_stationary_frame_s']:
                state=read_arm(a.arm_url)
                if all(abs(state['joint_median'][j]-cached['after']['joint_median'][j])<=2 for j in state['joint_median']):
                    visual=cached['visual'];reused=True
                    result['observations'].append({'before':cached['before'],'after':cached['after'],
                        'visual':visual,'reused_from_approach':True,'reuse_validation_read':state});save()
                    print('复用已验收的末次接近照片，检查闭爪条件。',flush=True)
            if not reused:visual,state=observe(False)
            old=journal['observations'][-1]['after']['joint_median'];now=state['joint_median']
            if any(abs(now[j]-old[j])>5 for j in now):raise ValueError('当前姿态已偏离接近终点')
            tcp=np.array(visual['tcp_board_cm']);obj=np.array(visual['T_board_object'])
            result['grasp_region']=motion['grasp_region'];result['motion_profile']=journal.get('motion_profile')
            grasp_offset_board=journal.get('grasp_offset_board_cm')
            result['grasp_offset_board_cm']=grasp_offset_board
            if not in_grasp_region(tcp,obj,motion['grasp_region'],grasp_offset_board=grasp_offset_board):
                raise ValueError('新图不满足本轮抓点的接触区域')
            model=nominal_model()
            pitch=model.forward({int(j):v for j,v in now.items()})['pitch_deg']
            if not pitch_in_profile(pitch,motion):raise ValueError('闭爪前俯仰不满足本次姿态范围')
            require_start_state(now,motion=motion)
            alignment_limit=motion.get('maximum_fixed_wrist_alignment_deg')
            if alignment_limit is not None:
                # The pitch-tool Y axis is the nominal closing direction for the fixed wrist.
                tool_rotation=robot_frame(now,model,'pitch_tool')[:3,:3]
                closing_axis=tool_rotation[:,1]
                box_width_axis=(observer.Y@obj)[:3,1]
                alignment=undirected_angle_deg(closing_axis,box_width_axis)
                result['preclosure_nominal_alignment_deg']=alignment
                result['preclosure_wrist_alignment_preview']=preview_wrist_alignment(
                    tool_rotation,box_width_axis,observer.Y[:3,2])
                if alignment>alignment_limit:
                    raise ValueError(f'闭爪前名义夹合方向偏离盒宽 {alignment:.1f}°，超过试验上限 {alignment_limit:g}°')
            up=-observer.Y[:3,2]
            def plan_lift(reading,height):
                joints=reading['joint_median']
                limits={r['id']:(r['min'],r['max']) for r in reading['status']['data']['servo_limits']}
                joint_bias=None
                if motion.get('adaptive_pitch_search') and journal['commands']:
                    last=journal['commands'][-1]
                    targets={str(row['id']):row['position'] for row in last['request'].get('targets',[])}
                    if last.get('task',{}).get('status')=='reached' and set(targets)=={'3','4','5','6'}:
                        errors={j:joints[j]-target for j,target in targets.items()}
                        if max(abs(v) for v in errors.values())<=12:
                            joint_bias={j:max(-8,min(8,v)) for j,v in errors.items()}
                return plan_verification_lift(model,joints,observer.tcp['offset_tool_cm'],limits,height,up,motion,
                                              joint_bias=joint_bias)
            if motion.get('adaptive_pitch_search'):
                result['preclosure_lift_preview']=plan_lift(state,-tcp[2]);save()
            if not a.execute:result['phase']='preview_only';return
            move({'1':520},'single authorized empirical closure; coarse position completion is not grasp confirmation',40)
            after_close=None
            if motion.get('reuse_preclosure_lift_baseline'):
                after_close=read_arm(a.arm_url)
                jaw=[next(v['position'] for v in sample['data']['joint_positions'] if v['id']==1)
                     for sample in after_close['pose_samples']]
            else:
                jaw=[]
                for _ in range(5):
                    readings=call(a.arm_url,'positions?refresh=true')
                    if readings.get('fresh') is not True or readings.get('source')!='serial':raise RuntimeError('夹爪反馈不是新串口读数')
                    jaw.append(next(v['position'] for v in readings['positions'] if v['id']==1));time.sleep(.25)
            result['jaw_samples']=jaw;save()
            if not stable_loaded(jaw):raise RuntimeError('闭合位置不在当前药盒的稳定观察区；不试提或追加收紧')
            if after_close is not None and can_reuse_lift_baseline(state,after_close,visual):
                baseline,state=visual,after_close
                result['lift_baseline_source']='preclosure_accepted_support_observation'
                result['postclosure_feedback']=after_close
                print('闭爪后夹爪及臂姿反馈稳定，复用闭爪前的支撑面观测。',flush=True)
            else:
                baseline,state=observe(True)
                result['lift_baseline_source']='postclosure_photo'
            if not -.7<=baseline['minimum_box_bottom_height_cm']<=.7:
                raise ValueError('试提前自由物体位姿未与支撑面吻合，无法独立验收')
            start_height=min(-tcp[2],-baseline['tcp_board_cm'][2])
            plan=plan_lift(state,start_height)
            result['lift_plan']=plan;save()
            move(plan['targets'],'single 2cm lift with bounded pitch change')
            result['phase']='holding_not_verified'
            save()
            offset=np.asarray(observer.tcp['offset_tool_cm'])
            def tcp_point(joints):
                frame=robot_frame(joints,model,'pitch_tool')
                return frame[:3,:3]@offset+frame[:3,3]
            start_point=tcp_point(state['joint_median'])
            def report_lift(plan,reading):
                actual=reading['joint_median']
                rise=float((tcp_point(actual)-start_point)@up)
                report={'command':plan['targets'],'expected':plan['expected_joint_positions'],
                        'actual':{j:actual[j] for j in plan['targets']},'model_total_rise_cm':rise,
                        'model_planned_step_rise_cm':float(np.asarray(plan['predicted_delta_cm'])@up)}
                result.setdefault('lift_joint_feedback',[]).append(report);save()
                print(f"试提舵机单位  指令 {report['command']}，预期 {report['expected']}，实读 {report['actual']}。"
                      f" 模型累计抬升 {rise:.2f} / 2.00 cm。",flush=True)
                return rise
            lifted_state=read_arm(a.arm_url)
            rise=report_lift(plan,lifted_state)
            joints=lifted_state['joint_median']
            if (motion.get('adaptive_pitch_search') and .3<2.-rise<=1.2
                    and stable_hold_feedback(lifted_state,lifted_state)):
                limits={r['id']:(r['min'],r['max']) for r in lifted_state['status']['data']['servo_limits']}
                bias={j:joints[j]-target for j,target in plan['targets'].items()}
                bias={j:max(-8,min(8,value)) for j,value in bias.items()}
                try:
                    correction=plan_step(model,joints,up*min(.8,2.-rise),plan['pitch_goal_deg'],
                                         offset,limits,start_height+rise,up,
                                         max(motion['minimum_nominal_pad_height_cm'],start_height+rise-.15),
                                         joint_bias=bias)
                    if not correction['feedback_detectable'] or correction['max_joint_delta']>25:
                        raise ValueError('补偿超出小步关节范围或小于可检测变化')
                except ValueError as error:
                    result['lift_correction_skipped']=str(error);save()
                    print(f'试提补偿规划未通过，按当前姿态进行视觉验收。{error}',flush=True)
                else:
                    result['lift_correction_plan']=correction;save()
                    move(correction['targets'],'试提关节反馈补偿')
                    report_lift(correction,read_arm(a.arm_url))
            verify_lift(baseline)
    except BaseException as e:
        result['error']=str(e)
        if result['phase']!='holding_not_verified':result['phase']='failed'
        raise
    finally:save();print('闭合与试提记录：',output,flush=True)

if __name__=='__main__':main()
