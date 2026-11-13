"""One entry point for the current-box commissioning workflow; no GPT calls.

PC currently owns the camera and dispatches the existing Pi HTTP executor.
This is not yet the final Pi-owned coordinator or a general bottle detector.
"""
import argparse,hashlib,json,time
from pathlib import Path
from datetime import datetime
from check_grasp_board import ROOT

def newest(pattern,start,predicate=lambda d:True):
    matches=[]
    for path in (ROOT/'vision_grasp_results').glob(pattern):
        if path.stat().st_mtime>=start:
            data=json.loads(path.read_text(encoding='utf-8'))
            if predicate(data):matches.append(path)
    if not matches:raise RuntimeError('本阶段没有生成对应的完成记录：'+pattern)
    return max(matches,key=lambda p:p.stat().st_mtime)

def approach_resume_reference(saved):
    if saved.get('gripper_close_sent') or any(c.get('task',{}).get('status')!='reached' for c in saved.get('commands',[])):
        raise ValueError('存在未知运动或已经闭爪，不能接续')
    valid=[p for p in saved['observations'] if p.get('visual')]
    if not valid:
        raise ValueError('没有已验收的近景观测')
    last=valid[-1]['after']
    expected=last['joint_median']
    if saved.get('commands'):
        task=saved['commands'][-1]['task']
        feedback=task.get('position_feedback') or {}
        last_image_at=last['pose']['data'].get('observed_at')
        last_move_at=feedback.get('observed_at') or task.get('updated_at')
        if last_image_at and last_move_at and datetime.fromisoformat(last_move_at)>datetime.fromisoformat(last_image_at):
            positions=feedback.get('joint_positions',{})
            if feedback.get('within_tolerance') is not True or not all(j in positions for j in expected):
                raise ValueError('最后一条已完成动作缺少可核对的关节反馈')
            return positions,'last_confirmed_move'
    return expected,'last_accepted_observation'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--camera-baudrate',type=int,default=115200)
    p.add_argument('--arm-url',default='http://127.0.0.1:18000/v1/arm')
    p.add_argument('--calibration',type=Path,required=True)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute',action='store_true',help='允许本盒一次接近、520闭合及试提')
    mode.add_argument('--preview',action='store_true',help='仅拍照、定位和预览抓取路径')
    p.add_argument('--result',type=Path)
    p.add_argument('--motion-profile',type=Path)
    p.add_argument('--fallback-motion-profile',type=Path)
    resume=p.add_mutually_exclusive_group()
    resume.add_argument('--resume-hover',type=Path,help='只接续同场景已完成的预接近，不重发旧动作')
    resume.add_argument('--resume-approach',type=Path,help='只接续同场景、所有运动结果已确认的视觉阶段')
    a=p.parse_args()
    if a.preview and (a.resume_hover or a.resume_approach):
        p.error('路径预览需要从观察位拍摄新场景')
    from vision_task import VisionTaskSession
    images=ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d')
    with VisionTaskSession(a.port,'med_003',images=images,baudrate=a.camera_baudrate) as camera:
        run_trial(a,camera)


def run_trial(a,camera):
    from grasp_motion_profile import load_motion_profile
    from observe_grasp_target import main as observe
    from register_current_scene import register
    from approach_grasp_candidate import main as approach
    from guarded_visual_approach import main as correct
    from verify_same_box_grasp import main as verify
    camera_stages={
        'observe_grasp_target.py':observe,
        'approach_grasp_candidate.py':approach,
        'guarded_visual_approach.py':correct,
        'verify_same_box_grasp.py':verify,
    }
    start=time.time();stages=[]
    out=a.result or ROOT/'vision_grasp_results'/('single_entry_trial_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    out.parent.mkdir(parents=True,exist_ok=True)
    motion=load_motion_profile(a.motion_profile)
    motion_args=[] if a.motion_profile is None else ['--motion-profile',str(a.motion_profile.resolve())]
    tracked_sources=(*camera_stages,'run_current_box_trial.py','grasp_controller.py','register_current_scene.py',
                     'board_marker_pose.py','detect_box_outline.py','fit_known_box_outline.py','fit_grasp_handeye.py',
                     'grasp_visual_observer.py','tcp_pose_planner.py','grasp_approach_policy.py')
    report={'stages':stages,'grasp_confirmed':False,
        'trial_conditions':{'mode':'execute' if a.execute else 'preview',
            'camera_port':a.port,'camera_baudrate':a.camera_baudrate,
            'motion_profile':motion,'calibration_sha256':hashlib.sha256(a.calibration.read_bytes()).hexdigest(),
            'core_source_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in tracked_sources}}}
    def save():
        out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    def select_motion(path,selected,selection):
        nonlocal motion,motion_args
        motion=selected
        motion_args=['--motion-profile',path] if path else []
        report['selected_motion_profile']=path
        report['motion_profile_selection']=selection
        report['trial_conditions']['motion_profile']=motion
        save()
    def run(script,*args):
        row={'script':script,'started_at':time.time()};stages.append(row)
        save()
        try:
            if script=='register_current_scene.py':
                registered=register(Path(args[0]),motion=motion)
                scene=json.loads(registered.read_text(encoding='utf-8'))
                report.update(registered_scene=str(registered),
                    nominal_scene_geometry=scene['scene_registration']['nominal_scene_geometry'],
                    board_pose_rmse_px=scene['scene_registration']['board_pnp_rmse_px'],
                    box_alignment_deg=scene['grasp_scope']['long_axis_to_radial_deg'])
            elif script=='approach_grasp_candidate.py':
                approach(list(map(str,args)),camera,on_plan=select_motion)
            else:
                camera_stages[script](list(map(str,args)),camera)
            row['returncode']=0
        except Exception as error:
            reason=f'{type(error).__name__}: {error}'
            row.update(returncode=1,error=reason)
            raise RuntimeError('本轮结束于'+script+'；原因：'+reason+'；保留现场状态，不自动恢复') from error
        finally:
            row['completed_at']=time.time()
            save()
    common=('--port',a.port,'--arm-url',a.arm_url)
    seed_args=[]
    if a.resume_approach:
        from collect_grasp_reference import read_arm
        from grasp_start_state import require_start_state
        seed=a.resume_approach.resolve();saved=json.loads(seed.read_text(encoding='utf-8'))
        if time.time()-seed.stat().st_mtime>3600:raise ValueError('视觉接续记录已过期')
        expected,reference_kind=approach_resume_reference(saved)
        registered=Path(saved['source']);hover=Path(saved['hover_handoff']['report'])
        hover_profile=json.loads(hover.read_text(encoding='utf-8')).get('motion_profile')
        saved_profile=saved.get('motion_profile')
        if saved_profile and hover_profile and Path(saved_profile).resolve()!=Path(hover_profile).resolve():
            raise ValueError('视觉接近和悬停记录的运动配置不一致，不能接续')
        if saved_profile or hover_profile:
            path=str(Path(saved_profile or hover_profile).resolve())
            select_motion(path,load_motion_profile(path),{'mode':'resume'})
        actual=read_arm(a.arm_url);require_start_state(actual['joint_median'],motion=motion)
        if any(abs(actual['joint_median'][j]-expected[j])>5 for j in ('2','3','4','5','6')):
            raise ValueError('现场关节反馈已偏离最后确认的位置，不能接续')
        report['approach_resume_reference']=reference_kind
        seed_args=['--seed-approach',str(seed)]
    elif a.resume_hover:
        from collect_grasp_reference import read_arm
        from grasp_start_state import require_start_state
        hover=a.resume_hover.resolve();saved=json.loads(hover.read_text(encoding='utf-8'))
        if not saved.get('hover_motion_completed'):raise ValueError('不能从未完成的预接近接续')
        if time.time()-hover.stat().st_mtime>3600:raise ValueError('悬停记录已超过本次现场接续时限，需要新场景准备')
        registered=Path(saved['source'])
        if saved.get('motion_profile'):
            path=str(Path(saved['motion_profile']).resolve())
            select_motion(path,load_motion_profile(path),{'mode':'resume'})
        actual=read_arm(a.arm_url);require_start_state(actual['joint_median'],motion=motion)
        expected=saved['captures'][-1]['after']['joint_median']
        if any(abs(actual['joint_median'][j]-expected[j])>5 for j in ('2','3','4','5','6')):
            raise ValueError('现场姿态已离开悬停点，不执行接续动作')
    else:
        run('observe_grasp_target.py',*common,'--calibration',a.calibration)
        # Register today's board before requiring a world-space box fit.
        observation=newest('*_commissioning.json',start,lambda d:d.get('ocr',{}).get('target_selection',{}).get('status')=='target_found')
        report['initial_observation']=str(observation)
        run('register_current_scene.py',observation)
        registered=observation.with_name(observation.stem+'_registered.json')
        execute_args=['--execute'] if a.execute else []
        fallback_args=['--fallback-motion-profile',a.fallback_motion_profile] if a.fallback_motion_profile else []
        run('approach_grasp_candidate.py',registered,*common,'--tcp-profile',ROOT/'calibration_boards/grasp_tcp_hover_candidate_20261105.json',*motion_args,*fallback_args,*execute_args)
        if not a.execute:
            report.update(status='path_preview_passed',hardware_motion_sent=False)
            save()
            print('新场景路径预览通过。记录：',out,flush=True)
            return
        hover=newest('hover_*.json',start,lambda d:Path(d.get('source','')).resolve()==registered.resolve() and d.get('hover_motion_completed'))
    run('guarded_visual_approach.py',*common,'--observation',registered,'--hover-report',hover,*seed_args,*motion_args,'--execute')
    approach_report=newest('visual_approach_*.json',start,lambda d:Path(d.get('source','')).resolve()==registered.resolve() and d.get('completed'))
    run('verify_same_box_grasp.py',approach_report,*common,'--execute')
    trial=newest('grasp_trial_*.json',start,lambda d:Path(d.get('source','')).resolve()==approach_report.resolve())
    result=json.loads(trial.read_text(encoding='utf-8'))
    report.update(verification=str(trial),grasp_confirmed=result.get('grasp_confirmed',False))
    save()
    print('整轮记录：',out)

if __name__=='__main__':main()
