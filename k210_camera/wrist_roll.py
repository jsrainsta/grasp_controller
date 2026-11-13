"""Measured servo-2 mapping and bounded empty-wrist motion."""
import math
import time
import uuid

import numpy as np

from fit_grasp_handeye import preview_wrist_alignment, robot_frame, transform
from grasp_start_state import WRIST_POSITION, require_start_state


def validate_wrist_roll(cal):
    def number(value):
        return type(value) in (int, float) and math.isfinite(value)

    if not isinstance(cal, dict) or cal.get('field_validated') is not True:
        raise ValueError('转腕须提供经现场核验的标定')
    scale = cal.get('degrees_per_count')
    counts = cal.get('position_range')
    pivot = cal.get('pivot_tool_cm')
    clearance = cal.get('minimum_sweep_tcp_height_cm')
    pitch_range = cal.get('sweep_pitch_range_deg')
    if (not number(scale) or not .05 <= abs(scale) <= 1
            or not isinstance(counts, list) or len(counts) != 2
            or any(type(v) is not int for v in counts)
            or not WRIST_POSITION-100 <= counts[0] < WRIST_POSITION < counts[1] <= WRIST_POSITION+100
            or not isinstance(pivot, list) or len(pivot) != 3
            or not all(number(v) for v in pivot)
            or not number(clearance) or not 3.5 <= clearance <= 8.5
            or not isinstance(pitch_range, list) or len(pitch_range)!=2
            or not all(number(v) for v in pitch_range)
            or not -95 <= pitch_range[0] < pitch_range[1] <= -55
            or type(cal.get('camera_rotates_with_wrist')) is not bool):
        raise ValueError('转腕标定参数无效或超出已采样范围')


def roll_transform(cal, position):
    if cal is None:
        return np.eye(4)
    if not cal['position_range'][0] <= position <= cal['position_range'][1]:
        raise ValueError('2号舵机反馈超出标定范围')
    theta = math.radians((position-WRIST_POSITION)*cal['degrees_per_count'])
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    pivot = np.array(cal['pivot_tool_cm'])
    return transform(R, pivot-R@pivot)


def wrist_target(cal, tool_rotation, box_width_axis, support_normal):
    preview = preview_wrist_alignment(tool_rotation, box_width_axis, support_normal)
    if preview['status'] != 'nominal_roll_preview':
        raise ValueError('无法从当前图像确定转腕方向')
    target = round(WRIST_POSITION+preview['relative_roll_deg']/cal['degrees_per_count'])
    roll_transform(cal, target)
    return target


def check_wrist_sweep(target, joints, limits, height, up, model, offset, cal):
    start = joints['2']
    if not limits['min'] <= target <= limits['max']:
        raise ValueError('转腕目标超出舵机限位')
    pitch = model.forward({int(j):v for j,v in joints.items()})['pitch_deg']
    if not cal['sweep_pitch_range_deg'][0] <= pitch <= cal['sweep_pitch_range_deg'][1]:
        raise ValueError('当前俯仰超出现场核验的空爪转腕范围')
    frame = robot_frame(joints, model, 'pitch_tool')
    R = frame[:3,:3]
    current = (roll_transform(cal,start)@np.r_[offset,1])[:3]
    low, high = min(start,target)-3, max(start,target)+3
    if not max(limits['min'],cal['position_range'][0]) <= low <= high <= min(limits['max'],cal['position_range'][1]):
        raise ValueError('转腕路径未给反馈容差保留标定和舵机限位余量')
    for count in range(low,high+1):
        point = (roll_transform(cal,count)@np.r_[offset,1])[:3]
        if height+float(up@R@(point-current)) < cal['minimum_sweep_tcp_height_cm']:
            raise ValueError('转腕扫过路径低于已核验的留空高度')


def move_empty_wrist(base, target, state, height, up, model, offset, cal, motion, reader, sender, log, save,
                     *, open_target=None):
    fresh = reader(base)
    before = fresh['joint_median']
    require_start_state(before,wrist=state['joint_median']['2'],motion=motion)
    if (any(abs(before[j]-state['joint_median'][j])>2 for j in before)
            or any(v>6 for v in fresh['joint_span'].values())
            or any(v>2 for v in fresh['joint_mad'].values())):
        raise RuntimeError('空爪转腕前姿态不稳定或偏离观测')
    if not fresh['status']['data']['capabilities']['writes_enabled']:
        raise RuntimeError('转腕运动未启用')
    limits = next(row for row in fresh['status']['data']['servo_limits'] if row['id']==2)
    check_wrist_sweep(target,before,limits,height,up,model,offset,cal)
    start = before['2']
    segments = max(1,(abs(target-start)+39)//40)
    opening = open_target is not None and before['1']>open_target
    if opening:
        jaw_limit=next(row for row in fresh['status']['data']['servo_limits'] if row['id']==1)
        if not jaw_limit['min']<=open_target<=jaw_limit['max']:
            raise ValueError('张爪目标超出1号舵机限位')
    for i in range(1,segments+1):
        waypoint=round(start+(target-start)*i/segments)
        if not fresh['status']['data']['capabilities']['writes_enabled']:
            raise RuntimeError('转腕运动未启用')
        jaw_goal=open_target if opening and i==1 else None
        after = send_wrist_waypoint(base,waypoint,fresh,reader,sender,log,save,cal,jaw_goal=jaw_goal)
        fresh = after
        stationary=('3','4','5','6') if opening else ('1','3','4','5','6')
        if any(abs(after['joint_median'][j]-before[j])>3 for j in stationary):
            raise RuntimeError('转腕期间其他关节累计偏移')
        require_start_state(after['joint_median'],wrist=waypoint,motion=motion)
    return fresh


def send_wrist_waypoint(base, target, fresh, reader, sender, log, save, cal, *, jaw_goal=None):
    before = fresh['joint_median']
    if abs(target-before['2'])>43:
        raise ValueError('转腕分段步幅超限')
    request = {'request_id':'wrist_align_'+uuid.uuid4().hex,
               'targets':[{'id':2,'position':target}], 'duration_ms':20,
               'position_tolerance':3}
    if jaw_goal is not None:
        request['targets'].append({'id':1,'position':jaw_goal})
    record = {'request':request, 'before':fresh}
    log.append(record)
    save()
    task = sender(base,'move',request)['task']
    record['task'] = task
    save()
    deadline = time.monotonic()+24
    while task['status']=='sent' and time.monotonic()<deadline:
        time.sleep(.25)
        task = sender(base,'tasks/'+task['task_id'])
        record['task'] = task
        save()
    if task['status']!='reached':
        raise RuntimeError('2号舵机动作未确认，停止本轮')
    after = reader(base)
    record['after'] = after
    save()
    joints = after['joint_median']
    roll_transform(cal,joints['2'])
    jaw_expected=before['1'] if jaw_goal is None else jaw_goal
    if (abs(joints['2']-target)>3
            or abs(joints['1']-jaw_expected)>3
            or any(abs(joints[j]-before[j])>3 for j in ('3','4','5','6'))
            or any(v>6 for v in after['joint_span'].values())
            or any(v>2 for v in after['joint_mad'].values())):
        raise RuntimeError('转腕后反馈不稳定或其他关节发生移动')
    return after
