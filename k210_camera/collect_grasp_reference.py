"""Capture one stationary board observation bracketed by actual joint readings.

No POST requests and no arm motion. Uses the unchanged local arm HTTP service.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time
import uuid

from autonomous_preflight import get_json
from check_grasp_board import inspect, ROOT
from receive_photo import receive_photos, choose_port


def read_arm(base):
    status = get_json(base, 'status')
    if status.get('ok') is not True or status.get('mode') != 'serial':
        raise RuntimeError('未取得实物状态')
    state = status['data']
    if (state.get('connection') != 'connected' or state.get('active_task_id') is not None
            or state.get('upload_in_progress') or state.get('limits_require_recheck')):
        raise RuntimeError('机械臂不满足空闲观测条件：' + str(state))
    samples = []
    for i in range(5):
        pose = get_json(base, 'pose?refresh=true')
        if pose.get('ok') is not True or pose.get('mode') != 'serial':
            details=pose.get('error_body',pose.get('error',pose))
            raise RuntimeError(f"未取得实物姿态（第{i+1}/5次，HTTP {pose.get('http_status','未报告')}）：{details}")
        p = pose['data']
        if p.get('fresh') is not True or p.get('source') != 'serial':
            raise RuntimeError('关节反馈不是新实物读数')
        samples.append(pose)
        if i < 4:
            time.sleep(.1)
    rows = [{v['id']: v['position'] for v in s['data']['joint_positions']} for s in samples]
    median = {str(j): int(statistics.median(r[j] for r in rows)) for j in range(1, 7)}
    span = {str(j): max(r[j] for r in rows)-min(r[j] for r in rows) for j in range(1, 7)}
    mad = {str(j): statistics.median(abs(r[j]-median[str(j)]) for r in rows) for j in range(1, 7)}
    return {'status': status, 'pose': pose, 'pose_samples': samples,
            'joint_median': median, 'joint_span': span, 'joint_mad': mad,
            'received_at': datetime.now(timezone.utc).isoformat()}


def save_observation(before, after, image):
    profile = ROOT/'calibration_boards'/'charuco_7x5_print96_v1.json'
    try:
        board = inspect(Path(image), profile)
    except Exception as error:
        board = {'image': str(image), 'reference_candidate': False, 'error': str(error)}
    a, b = before['joint_median'], after['joint_median']
    delta = {str(i): b[str(i)]-a[str(i)] for i in range(1, 7)}
    stable = (all(abs(v) <= 2 for v in delta.values())
              and all(v <= 6 for r in (before, after) for v in r['joint_span'].values())
              and all(v <= 2 for r in (before, after) for v in r['joint_mad'].values()))
    record = {'type': 'stationary_board_arm_observation', 'version': 2,
              'before': before, 'after': after, 'board': board,
              'joint_drift': delta, 'joint_bracket_stable': stable,
              'stability_policy': {'samples_per_burst': 5, 'max_median_drift': 2,
                                   'max_burst_span': 6, 'max_burst_mad': 2},
              'hardware_exposure_sync': False, 'robot_transform_calibrated': False,
              'reference_candidate': bool(stable and board['reference_candidate']),
              'arm_motion_commands_sent': False}
    out = ROOT/'calibration_checks'/(Path(image).stem+'_'+uuid.uuid4().hex[:8]+'_paired.json')
    out.parent.mkdir(exist_ok=True)
    with out.open('x', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    print('拍照前后关节中位值差：', delta)
    print('配对记录：', out)
    return out, record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-url', required=True)
    parser.add_argument('--port')
    args = parser.parse_args()
    before = read_arm(args.arm_url)
    port = args.port or choose_port()
    image = receive_photos(port, 115200, ROOT/'calibration_images'/datetime.now().strftime('%Y%m%d'),
                           once=True, request_capture=True)
    if image is None:
        raise RuntimeError('没有取得本次图像')
    after = read_arm(args.arm_url)
    out, record = save_observation(before, after, image)
    if not record['reference_candidate']:
        raise RuntimeError('本次配对未通过质量检查，未用于机器人定位')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('本步结束：', error)
        raise SystemExit(1)
