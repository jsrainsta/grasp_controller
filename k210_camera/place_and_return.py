"""Place the currently held trial box back at its recorded support pose.

Explicit, same-scene preparation only. Never reconnects, resets, releases busy,
retries unknown writes, or starts a grasp. Support confirmation is mandatory.
"""
import argparse,json,time,uuid,math
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from grasp_controller import load_config,resolve_port,ControllerLock
from check_grasp_board import ROOT
from collect_grasp_reference import read_arm
from sample_grasp_handeye import call,nominal_model
from grasp_visual_observer import GraspVisualObserver
from vision_task import VisionTaskSession
from jpeg_integrity import check_jpeg
from tcp_pose_planner import plan_step
from fit_grasp_handeye import robot_frame
from grasp_start_state import require_start_state
from grasp_motion_profile import load_motion_profile

def supported_bottom_range(visual):
    T=np.array(visual['T_board_object'])
    bottom=np.array([[-5.25,-1.7,2.5],[5.25,-1.7,2.5],[5.25,1.7,2.5],[-5.25,1.7,2.5]])
    heights=-((T[:3,:3]@bottom.T+T[:3,3:]).T)[:,2]
    return float(heights.min()),float(heights.max())

def placement_targets(trial,actual,limits):
    if trial.get('grasp_confirmed') is not True:raise ValueError('记录不是已完成持物验收的试验')
    if len(trial['commands'])!=2 or any(c.get('task',{}).get('status')!='reached' for c in trial['commands']):
        raise ValueError('持物记录存在未知或额外动作')
    expected=trial['observations'][-1]['after']['joint_median']
    if not 480<=actual['1']<=505 or any(abs(actual[j]-expected[j])>5 for j in ('2','3','4','5','6')):
        raise ValueError('当前持物姿态已改变，不重放放置路径')
    if trial.get('lift_baseline_source')=='preclosure_accepted_support_observation':
        base=trial['postclosure_feedback']['joint_median']
    else:
        base=trial['observations'][1]['after']['joint_median']
    command={str(v['id']):v['position'] for v in trial['commands'][-1]['request']['targets']}
    result={}
    for j in ('3','4','5','6'):
        bias=actual[j]-command[j]
        if abs(bias)>10:raise ValueError('当前关节偏差超过本次放置限额')
        result[j]=round(base[j]-bias)
        if not limits[int(j)][0]<=result[j]<=limits[int(j)][1] or abs(result[j]-actual[j])>90:
            raise ValueError('回原支撑姿态的目标不满足限位/步幅')
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--confirm-support-unchanged',action='store_true');p.add_argument('--execute',action='store_true')
    a=p.parse_args();config=load_config(a.config);trial=json.loads(a.trial.read_text(encoding='utf-8'))
    motion=load_motion_profile(config.get('motion_profile'))
    if not a.execute:raise ValueError('此入口需要--execute；不会仅因存在记录就操作硬件')
    if not a.confirm_support_unchanged:raise ValueError('须确认书本/参照纸/底座未移动且原支撑处可放回物体')
    report={'type':'same_scene_place_and_return','trial':str(a.trial.resolve()),'commands':[],'observations':[],
            'phase':'starting','completed':False,'support_confirmed_unchanged':True,
            'record_age_s':time.time()-a.trial.stat().st_mtime}
    out=Path(config['results_dir'])/('place_return_'+uuid.uuid4().hex+'.json');out.parent.mkdir(parents=True,exist_ok=True)
    def save():out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    def move(targets,purpose):
        read_arm(config['arm_url'])
        body={'request_id':'place_return_'+uuid.uuid4().hex,'targets':[{'id':int(j),'position':int(v)} for j,v in targets.items()],'duration_ms':20}
        c={'purpose':purpose,'request':body};report['commands'].append(c);save()
        r=call(config['arm_url'],'move',body);c['response']=r;save();task=r['task'];deadline=time.monotonic()+24
        while task['status']=='sent' and time.monotonic()<deadline:
            time.sleep(.25);task=call(config['arm_url'],'tasks/'+task['task_id'])
        c['task']=task;save()
        if task['status']!='reached':raise RuntimeError('动作结果未确认；不重发、不松爪、不复位：'+str(task))
    with ControllerLock(Path(config['results_dir'])/'controller.lock'):
        try:
            state=read_arm(config['arm_url']);limits={v['id']:(v['min'],v['max']) for v in state['status']['data']['servo_limits']}
            targets=placement_targets(trial,state['joint_median'],limits)
            old=json.loads(Path(trial['source']).read_text(encoding='utf-8'))
            scene=json.loads(Path(old['source']).read_text(encoding='utf-8'))
            observer=GraspVisualObserver(Path(old['source']),Path(old['tcp_profile']));observer.restore_grounded_journal(old)
            with VisionTaskSession(resolve_port(config['camera_port']),'med_003',images=ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d'),
                baudrate=config['camera_baudrate'],
                runner=SimpleNamespace(target={'id':'med_003','name':'阿昔洛韦乳膏'})) as camera:
                def observe():
                    for _ in range(2):
                        before=read_arm(config['arm_url']);capture=camera.capture('place_obs_'+uuid.uuid4().hex)
                        if capture.get('status')!='captured':raise RuntimeError('放置拍照失败')
                        after=read_arm(config['arm_url']);path=Path(capture['image_path'])
                        if not check_jpeg(path)['valid']:continue
                        if any(abs(before['joint_median'][j]-after['joint_median'][j])>3 for j in before['joint_median']):raise RuntimeError('拍照时姿态改变')
                        v=observer.estimate(path,after['joint_median'],held=True)
                        report['observations'].append({'capture':capture,'before':before,'after':after,'visual':v});save()
                        return v,after
                    raise RuntimeError('两次图像未通过检查')
                visual,state=observe()
                if not .3<=visual['minimum_box_bottom_height_cm']<=3.5:raise ValueError('物体当前高度与记录不符，拒绝盲放')
                targets=placement_targets(trial,state['joint_median'],limits)
                move(targets,'return held box to independently observed original support pose')
                visual,state=observe()
                low,high=supported_bottom_range(visual)
                if low<-.3 or high>.3 or high-low>.4:raise ValueError('盒底四角未回到支撑面附近，不松爪或继续下压')
                report['phase']='support_pose_observed';save()
                move({'1':385},'open only after support-plane observation')
                visual,state=observe();require_start_state(state['joint_median'],motion=motion)
                model=nominal_model();positions=state['joint_median'];pitch=model.forward({int(j):v for j,v in positions.items()})['pitch_deg']
                up=-observer.Y[:3,2];height=-visual['tcp_board_cm'][2]
                raise_plan=plan_step(model,positions,up*2.,pitch,observer.tcp['offset_tool_cm'],limits,height,up,1.5)
                move(raise_plan['targets'],'raise empty pads clear of placed box')
                state=read_arm(config['arm_url']);positions=state['joint_median'];startT=robot_frame(positions,model,'pitch_tool')
                offset=np.array(observer.tcp['offset_tool_cm']);start_point=startT[:3,:3]@offset+startT[:3,3]
                # Relative model displacement anchored by the last independent
                # image; never claim this is an absolute Cartesian measurement.
                clearance=height+float((start_point-(robot_frame(report['observations'][-1]['after']['joint_median'],model,'pitch_tool')[:3,:3]@offset+
                    robot_frame(report['observations'][-1]['after']['joint_median'],model,'pitch_tool')[:3,3]))@up)
                goal={**scene['after']['joint_median'],**{j:max(limits[int(j)][0],min(limits[int(j)][1],scene['after']['joint_median'][j])) for j in ('3','4','5','6')}}
                count=math.ceil(max(abs(goal[j]-positions[j]) for j in ('3','4','5','6'))/90)
                for f in np.linspace(0,1,61):
                    q={**positions,**{j:round(positions[j]+(goal[j]-positions[j])*f) for j in ('3','4','5','6')}}
                    T=robot_frame(q,model,'pitch_tool');point=T[:3,:3]@offset+T[:3,3]
                    if clearance+float((point-start_point)@up)<3.5:raise ValueError('返回观察位的路径留空不足，保持当前空爪位置')
                for i in range(1,count+1):
                    move({j:round(positions[j]+(goal[j]-positions[j])*i/count) for j in ('3','4','5','6')},'return empty arm to observation pose')
                report.update(phase='placed_and_observation_ready',completed=True,final=read_arm(config['arm_url']));save()
                print('已放回支撑处并返回观察位；未开始下一轮抓取。',flush=True)
        except BaseException as e:report.update(phase='failed',error=str(e));raise
        finally:save();print('放置记录：',out,flush=True)

if __name__=='__main__':main()
