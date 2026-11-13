"""Collect three empty-wrist roll samples. Only servo 2 may move."""
import argparse
import json
import time
import uuid
from pathlib import Path

from collect_grasp_reference import read_arm
from grasp_controller import ControllerLock, load_config
from grasp_motion_profile import load_motion_profile
from grasp_start_state import WRIST_POSITION, require_start_state
from sample_grasp_handeye import call


def collect(config, prompt=input, reader=read_arm, sender=call, *, span=40):
    directory=Path(config['results_dir'])
    path=directory/f'wrist_calibration_{uuid.uuid4().hex[:12]}.json'
    result={'type':'empty_wrist_calibration_samples','status':'in_progress',
            'samples':[],'commands':[],'calibration_complete':False}
    base=config['arm_url']
    motion=load_motion_profile(config.get('motion_profile'))

    def save():
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')

    with ControllerLock(directory/'controller.lock'):
        try:
            initial=reader(base)
            start=initial['joint_median']
            require_start_state(start,motion=motion)
            limits=next(row for row in initial['status']['data']['servo_limits'] if row['id']==2)
            if not limits['min']<=WRIST_POSITION-span<WRIST_POSITION+span<=limits['max']:
                raise RuntimeError('标定范围超出二号舵机限位')

            def read_stationary():
                state=reader(base)
                joints=state['joint_median']
                if (any(abs(joints[j]-start[j])>3 for j in ('1','3','4','5','6'))
                        or any(span>6 for span in state['joint_span'].values())
                        or any(mad>2 for mad in state['joint_mad'].values())):
                    raise RuntimeError('关节反馈不稳定或其他关节发生移动')
                return state

            def move_to(target):
                state=read_stationary()
                start=state['joint_median']['2']
                segments=max(1,(abs(target-start)+39)//40)
                for i in range(1,segments+1):
                    state=move_wrist(round(start+(target-start)*i/segments))
                return state

            def move_wrist(target):
                before=read_stationary()
                if abs(before['joint_median']['2']-target)>46:
                    raise RuntimeError('二号舵机偏离预期，当前步幅过大')
                request={'request_id':f'wrist_cal_{uuid.uuid4().hex}',
                         'targets':[{'id':2,'position':target}],'duration_ms':20,
                         'position_tolerance':6}
                command={'request':request}
                result['commands'].append(command)
                save()
                task=sender(base,'move',request)['task']
                deadline=time.monotonic()+20
                while task['status']=='sent' and time.monotonic()<deadline:
                    time.sleep(.25)
                    task=sender(base,'tasks/'+task['task_id'])
                command['task']=task
                save()
                if task['status']!='reached':
                    raise RuntimeError('二号舵机动作未确认到位')
                state=read_stationary()
                if abs(state['joint_median']['2']-target)>6:
                    raise RuntimeError('二号舵机稳定反馈超出目标容差')
                return state

            def sample(label,state):
                actual=state['joint_median']['2']
                row={'label':label,'servo_2_actual':actual,'feedback':state}
                result['samples'].append(row)
                save()
                name=prompt(f'{label}，二号实读 {actual}。固定机位拍照后输入文件名（可直接回车，q退出）：').strip()
                if name.lower()=='q':
                    raise KeyboardInterrupt
                after=read_stationary()
                if abs(after['joint_median']['2']-actual)>3:
                    raise RuntimeError('拍照前后二号舵机位置发生变化')
                row.update(photo_name=name,feedback_after=after)
                save()

            sample('1 中位',read_stationary())
            sample('2 低读数',move_to(WRIST_POSITION-span))
            move_to(WRIST_POSITION)
            sample('3 高读数',move_to(WRIST_POSITION+span))
            move_to(WRIST_POSITION)
            result['status']='samples_collected'
        except BaseException as error:
            result.update(status='incomplete',error=str(error) or type(error).__name__)
            raise
        finally:
            save()
            print('二号舵机采样记录：',path.resolve(),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--confirm-empty-clearance',action='store_true',
                        help='确认空爪、机械臂静止且夹爪整个转动范围内无障碍')
    parser.add_argument('--span',type=int,choices=(40,60,80,100),default=40,
                        help='相对496中位的采样幅度，单位为舵机位置计数')
    args=parser.parse_args()
    if not args.confirm_empty_clearance:
        parser.error('先现场确认空爪和整个转腕范围的余量')
    collect(load_config(args.config),span=args.span)


if __name__=='__main__':
    main()
