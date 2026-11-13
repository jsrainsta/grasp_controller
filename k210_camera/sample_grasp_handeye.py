"""Bounded, high-position calibration sampling. Explicit --execute enables moves.

No gripper/roll commands, no reset/reconnect/release, no retry of a move.
Calibration commissioning helper on the PC; not the production Pi controller.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import time
from types import SimpleNamespace
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import uuid

from check_grasp_board import ROOT
from collect_grasp_reference import read_arm, save_observation
from vision_task import VisionTaskSession


def call(base, path, body=None):
    data = None if body is None else json.dumps(body).encode('utf-8')
    req = Request(base.rstrip('/')+'/'+path, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=10) as f:
            result = json.load(f)
    except HTTPError as error:
        raise RuntimeError('HTTP '+str(error.code)+': '+error.read().decode('utf-8',errors='replace')) from error
    if result.get('ok') is not True or result.get('mode') != 'serial':
        raise RuntimeError(str(result))
    return result['data']


def nominal_model():
    package = ROOT.parent/'arm_service_review'/'received_20261105'
    spec = importlib.util.spec_from_file_location('reviewed_nominal_kinematics', package/'arm_service'/'kinematics.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = json.loads((package/'config.serial.integration.json').read_text(encoding='utf-8'))
    return module.KinematicsModel(config['kinematics'])


def plan():
    return [
        {'3':181, '4':150, '5':409, '6':471},
        {'3':181, '4':165, '5':409, '6':521},
        {'3':156, '4':130, '5':409, '6':496},
        {'3':206, '4':130, '5':409, '6':496},
        {'3':181, '4':150, '5':434, '6':471},
        {'3':181, '4':165, '5':434, '6':521},
        {'3':181, '4':130, '5':409, '6':496},
    ]


def check_path(model, start, targets, limits):
    # Only a conservative nominal tool-height/envelope check; not collision sensing.
    begin = {int(k): int(v) for k, v in start.items()}
    end = dict(begin)
    for key, value in targets.items():
        j = int(key)
        if j not in (3,4,5,6) or abs(value-begin[j]) > 65:
            raise RuntimeError('标定步幅或关节不符合本次计划')
        if not limits[j][0] <= value <= limits[j][1]:
            raise RuntimeError('标定目标超出接口限位')
        end[j] = value
    heights = []
    for step in range(21):
        sample = {j: round(begin[j]+(end[j]-begin[j])*step/20) for j in begin}
        model.check_nominal(sample)
        pose = model.forward(sample)
        if pose['z_cm'] < -3 or not 9 <= pose['x_cm'] <= 22 or abs(pose['y_cm']) > 5:
            raise RuntimeError('超出本次高位标定的名义范围')
        heights.append(pose['z_cm'])
    return min(heights)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-url', default='http://127.0.0.1:18000/v1/arm')
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    model = nominal_model()
    observation = read_arm(args.arm_url)
    start = observation['joint_median']
    expected = {'1':385, '2':496, '3':181, '4':126, '5':409, '6':496}
    if any(abs(start[j]-v) > 8 for j,v in expected.items()):
        raise RuntimeError('起始状态与今天的空载高位基准不符，未发送动作')
    limits = {v['id']:(v['min'],v['max']) for v in observation['status']['data']['servo_limits']}
    prior = start
    for i, target in enumerate(plan(), 1):
        height = check_path(model, prior, target, limits)
        print('预检', i, target, '路径名义最低Z：', round(height,2), flush=True)
        prior = {**prior, **target}
    if not args.execute:
        print('仅检查计划，未发送动作或拍照。')
        return
    directory = ROOT/'calibration_sessions'/('handeye_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    directory.mkdir(parents=True)
    manifest = {'type':'commissioning_handeye_samples', 'observations':[], 'commands':[],
                'completed':False, 'robot_transform_calibrated':False, 'started_at':datetime.now(timezone.utc).isoformat()}
    def save():
        (directory/'session.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    # Capture-only mode never loads OCR models.
    runner = SimpleNamespace(target={'id':'med_003', 'name':'阿昔洛韦乳膏'})
    try:
        with VisionTaskSession(args.port, 'med_003', images=ROOT/'calibration_images'/datetime.now().strftime('%Y%m%d'), runner=runner) as camera:
            def capture(label):
                for attempt in range(2):
                    before = read_arm(args.arm_url)
                    r = camera.capture('cal_'+uuid.uuid4().hex)
                    if r['status'] != 'captured':
                        raise RuntimeError('采传失败：'+str(r))
                    after = read_arm(args.arm_url)
                    path, record = save_observation(before, after, Path(r['image_path']))
                    manifest['observations'].append({'label':label, 'attempt':attempt+1, 'path':str(path),
                                                     'accepted':record['reference_candidate']})
                    save()
                    if record['reference_candidate']:
                        return record
                    print('本姿态观测未通过；最多补拍一次，不重发动作。', flush=True)
                raise RuntimeError('该视角两次观测均未通过，结束采集，不再运动')
            current = capture('baseline')
            for i, target in enumerate(plan(),1):
                before = read_arm(args.arm_url)
                s = before['status']['data']
                if s['capabilities'].get('writes_enabled') is not True:
                    raise RuntimeError('当前未允许运动')
                if any(abs(before['joint_median'][j]-start[j]) > 3 for j in ('1','2')):
                    raise RuntimeError('夹爪或腕轴状态已变，结束采集')
                check_path(model, before['joint_median'], target, limits)
                body = {'request_id':'handeye_'+uuid.uuid4().hex,
                        'targets':[{'id':int(j),'position':v} for j,v in target.items()], 'duration_ms':20}
                command = {'body':body, 'response':None, 'task':None}
                manifest['commands'].append(command)
                save()  # Persist the exact ID BEFORE transmitting. Never replay automatically.
                result = call(args.arm_url,'move',body)
                command['response'] = result
                save()
                task = result['task']
                print('标定动作', i, '任务：', task['task_id'], flush=True)
                deadline = time.monotonic()+22
                while task['status'] == 'sent' and time.monotonic() < deadline:
                    time.sleep(.3)
                    task = call(args.arm_url,'tasks/'+task['task_id'])
                command['task'] = task
                save()
                if task['status'] != 'reached':
                    raise RuntimeError('运动未确认到位，结束采集：'+str(task))
                time.sleep(.8)
                current = capture('pose_'+str(i))
            manifest['completed'] = True
    except BaseException as error:
        manifest['error'] = str(error)
        raise
    finally:
        save()
        print('采集记录：', directory/'session.json', flush=True)


if __name__ == '__main__':
    main()
