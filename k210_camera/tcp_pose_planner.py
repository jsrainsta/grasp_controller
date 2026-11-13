"""Bounded nominal TCP/pitch planner; physical progress needs visual feedback."""
import math
import numpy as np
from fit_grasp_handeye import robot_frame
from plan_fixed_box_grasp import ik


def plan_step(model,joints,delta,target_pitch,tcp_offset,limits,height,up,minimum_height,*,joint_bias=None,previous_joint_delta=None):
    current=robot_frame(joints,model,'pitch_tool')
    offset=np.array(tcp_offset,np.float64)
    tcp=current[:3,:3]@offset+current[:3,3]
    desired=tcp+np.array(delta)
    yaw=math.atan2(desired[1],desired[0]);theta=math.radians(target_pitch)
    for _ in range(6):
        c,s,ct,st=math.cos(yaw),math.sin(yaw),math.cos(theta),math.sin(theta)
        R=np.array([[c*ct,-s,-c*st],[s*ct,c,-s*st],[st,0,ct]])
        tool=desired-R@offset;yaw=math.atan2(tool[1],tool[0])
    start={str(j):int(v) for j,v in joints.items()}
    for j in ('3','4','5','6'):
        lo,hi=limits[int(j)]
        if not lo-10<=start[j]<=hi+10:raise ValueError('实际反馈偏离关节边界超过10单位，需要检查状态')
    choices=[]
    for solution in ik(tool,target_pitch,model.to_config()):
        target=solution['illustrative_joint_positions_not_for_execution']
        if any(not limits[int(j)][0]<=v<=limits[int(j)][1] for j,v in target.items()):continue
        bias=dict(joint_bias or {})
        if previous_joint_delta is not None:
            bias={j:b for j,b in bias.items() if b
                  and previous_joint_delta.get(j,0)*(target[j]-start[j])>0
                  and previous_joint_delta[j]*(target[j]-b-start[j])>0}
        command={j:v-bias.get(j,0) for j,v in target.items()}
        if any(not limits[int(j)][0]<=v<=limits[int(j)][1] for j,v in command.items()):continue
        maximum=max(abs(command[j]-start[j]) for j in command)
        if maximum>90 or max(abs(target[j]-start[j]) for j in target)>90:continue
        end={**start,**target};T=robot_frame(end,model,'pitch_tool')
        model.check_nominal({int(j):v for j,v in end.items()})
        end_tcp=T[:3,:3]@offset+T[:3,3]
        error=float(np.linalg.norm(end_tcp-desired))
        if error>.15:continue
        min_height=height
        # Both the uncorrected response to the command and the endpoint
        # predicted with the last measured bias must clear the support.
        for endpoint in (target,command) if bias else (target,):
            for f in np.linspace(0,1,41):
                mid={**start,**{j:round(start[j]+(endpoint[j]-start[j])*f) for j in endpoint}}
                if any(not min(limits[int(j)][0],start[j])<=mid[j]<=max(limits[int(j)][1],start[j]) for j in endpoint):
                    raise ValueError('路径扩大了起始反馈越界')
                M=robot_frame(mid,model,'pitch_tool');point=M[:3,:3]@offset+M[:3,3]
                min_height=min(min_height,height+float((point-tcp)@up))
        if min_height<minimum_height:continue
        choices.append((sum((target[j]-start[j])**2 for j in target),{
            'planner':'client_nominal_tcp_and_fixed_pitch','targets':command,
            'expected_joint_positions':target,'observed_joint_bias_used':bias,
            'max_joint_delta':maximum,'feedback_detectable':maximum>=6,
            'predicted_delta_cm':(end_tcp-tcp).tolist(),'rounding_error_cm':error,
            'pitch_goal_deg':target_pitch,'nominal_minimum_tcp_height_cm':min_height,
            'joint_limit_margin':min(min(v-limits[int(j)][0],limits[int(j)][1]-v)
                                     for endpoint in (target,command) for j,v in endpoint.items()),
            'physical_accuracy_verified':False}))
    if not choices:raise ValueError('本步固定俯仰TCP规划无可用受限逆解')
    return min(choices,key=lambda c:c[0])[1]

def choose_step(model,joints,delta,tcp_offset,limits,height,up,minimum_height,preferred_pitch=-70,pitch_range=(-85,-55),*,joint_bias=None,previous_joint_delta=None):
    """Prefer a feasible pitch with reserve, instead of the first legal pitch."""
    pitch=model.forward({int(j):v for j,v in joints.items()})['pitch_deg']
    angles=[pitch+float(np.clip(preferred_pitch-pitch,-6,6)),pitch,pitch-3,pitch+3,pitch-6,pitch+6]
    angles=list(dict.fromkeys(round(float(v),2) for v in angles if pitch_range[0]<=v<=pitch_range[1]))
    failures=[]
    for scale in (1.,.5):
        feasible=[]
        for theta in angles:
            try:plan=plan_step(model,joints,np.array(delta)*scale,theta,tcp_offset,limits,height,up,minimum_height,
                               joint_bias=joint_bias,previous_joint_delta=previous_joint_delta)
            except ValueError as error:failures.append(str(error));continue
            if not plan['feedback_detectable']:continue
            margin=plan['joint_limit_margin']
            score=5*max(0,20-margin)+2*abs(theta-preferred_pitch)+.1*abs(theta-pitch)+.03*plan['max_joint_delta']
            feasible.append((score,plan))
        if feasible:
            result=min(feasible,key=lambda p:p[0])[1]
            result['pitch_selection']='joint_limit_reserve_then_motion_cost';result['requested_step_scale']=scale
            return result
    raise ValueError('位置/俯仰候选均无受限路径：'+str(failures))


def plan_verification_lift(model, joints, tcp_offset, limits, height, up, motion,*,joint_bias=None):
    current_pitch=model.forward({int(j):v for j,v in joints.items()})['pitch_deg']
    adaptive=motion.get('adaptive_pitch_search')
    pitch_offsets=(0,.5,-.5,1.,-1.,2.,-2.,3.,-3.,4.,-4.) if adaptive else (0,)
    low,high=motion['nominal_pitch_range_deg']
    best=None
    for offset in pitch_offsets:
        goal_pitch=current_pitch+offset
        if offset and not low<=goal_pitch<=high:
            continue
        try:
            plan=plan_step(model,joints,up*2.,goal_pitch,tcp_offset,limits,height,up,
                           motion['minimum_nominal_pad_height_cm'],joint_bias=joint_bias)
        except ValueError:
            continue
        if not adaptive or plan['joint_limit_margin']>=10:
            return plan
        if best is None or plan['joint_limit_margin']>best['joint_limit_margin']:
            best=plan
    if best is not None:
        return best
    raise ValueError('从当前抓点无法规划2 cm试提，已检查允许的俯仰候选')
