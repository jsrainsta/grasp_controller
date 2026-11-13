"""Shared validated motion-profile loading; no hardware access."""
import copy,json
from pathlib import Path

LEGACY_REGION={'version':'same_box_trial_v2','along_length_cm':1.5,'across_width_cm':.45,
    'tcp_height_range_cm':[2.2,3.0],'tcp_height_target_cm':2.6,
    'basis':'Legacy current-box holding trial, not grasp quality acceptance.'}
DEFAULT_FIXED_WRIST_ALIGNMENT_DEG=12

def load_motion_profile(path=None):
    if path is None:return {'version':'legacy-v02','grasp_region':copy.deepcopy(LEGACY_REGION),
        'preferred_nominal_pitch_deg':-70,'nominal_pitch_range_deg':[-85,-55],
        'pregrasp_height_cm':6.,'pregrasp_min_top_clearance_cm':2.8,
        'minimum_nominal_pad_height_cm':1.9,'maximum_near_corrections':10,
        'maximum_near_step_cm':2.6,'reuse_fresh_stationary_frame_s':0,'maximum_coarse_joint_delta':80,'whole_path_precheck':False}
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('object_id')!='med_003' or data.get('enabled_for_execution') is not True:
        raise ValueError('此动作配置未允许当前药盒试验')
    if 'open_jaw_minimum' in data:
        minimum=data['open_jaw_minimum']
        if type(minimum) is not int or not 370<=minimum<=379:
            raise ValueError('空爪开度下限超出本盒试验范围')
    if 'maximum_fixed_wrist_alignment_deg' in data:
        limit=data['maximum_fixed_wrist_alignment_deg']
        if type(limit) not in (int,float) or not DEFAULT_FIXED_WRIST_ALIGNMENT_DEG<=limit<=18:
            raise ValueError('固定腕轴对齐角度超出本盒试验范围')
    r=data['grasp_region'];lo,hi=r['tcp_height_range_cm'];target=r['tcp_height_target_cm']
    if not 1.5<=lo<=target<=hi<=3.0:raise ValueError('抓点高度配置无效')
    if not 0<r['along_length_cm']<=1.5 or not 0<r['across_width_cm']<=.45:raise ValueError('抓取区域超出当前盒试验范围')
    a,b=data['nominal_pitch_range_deg']
    if not -95<=a<=data['preferred_nominal_pitch_deg']<=b<=-55:raise ValueError('俯仰配置无效')
    if not 1<=data['maximum_near_corrections']<=10:raise ValueError('纠偏预算无效')
    if not 60<=data['maximum_coarse_joint_delta']<=100:raise ValueError('粗接近关节步幅无效')
    if not 0<data['maximum_near_step_cm']<=3.2:raise ValueError('末端步幅无效')
    if not 1.3<=data['minimum_nominal_pad_height_cm']<=lo:raise ValueError('路径高度门槛无效')
    if not 4.2<=data['pregrasp_height_cm']<=8.5 or not 1.7<=data['pregrasp_min_top_clearance_cm']<=3:
        raise ValueError('预抓取余量配置无效')
    return data

def pitch_in_profile(pitch,profile,feedback_tolerance_deg=1.2):
    lo,hi=profile['nominal_pitch_range_deg']
    return lo-feedback_tolerance_deg<=pitch<=hi+feedback_tolerance_deg
