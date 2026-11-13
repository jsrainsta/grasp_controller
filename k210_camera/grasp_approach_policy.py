"""Position-only approach and contact checks for the commissioning box."""
import numpy as np

GRASP_REGION={'version':'same_box_trial_v2','along_length_cm':1.5,'across_width_cm':.45,
    'tcp_height_range_cm':[2.2,3.0],'tcp_height_target_cm':2.6,
    'basis':'Current 10.5cm box: middle 3cm grasp band; a trial parameter, holding still requires independent lift verification.'}

def grasp_point_board(obj,grasp_offset_board=None):
    point=np.array(obj[:3,3],dtype=float)
    if grasp_offset_board is not None:
        offset=np.asarray(grasp_offset_board,dtype=float)
        axis=np.asarray(obj[:2,0],dtype=float)
        axis_norm=np.linalg.norm(axis)
        if offset.shape!=(3,) or not np.isfinite(offset).all() or axis_norm<1e-8:
            raise ValueError('偏移抓点或盒长方向无效')
        axis=axis/axis_norm
        shift=float(offset[:2]@axis)
        if abs(shift)>1.2+1e-6:
            raise ValueError('偏移抓点超出本盒规划范围')
        point[:2]+=axis*shift
    return point


def in_grasp_region(tcp,obj,region=None,*,grasp_offset_board=None):
    region=GRASP_REGION if region is None else region
    tcp=np.asarray(tcp,float);obj=np.asarray(obj,float)
    e=obj[:3,:3].T@(grasp_point_board(obj,grasp_offset_board)-tcp)
    # A shifted band must still stay well inside the 10.5 cm box face.
    from_center=float(obj[:3,0]@(obj[:3,3]-tcp))
    lo,hi=region['tcp_height_range_cm']
    return bool(abs(from_center)<=2.7 and abs(e[0])<=region['along_length_cm']
                and abs(e[1])<=region['across_width_cm'] and lo<=-tcp[2]<=hi)

def approach_decision(tcp_board,T_board_object,motion_profile=None,*,grasp_offset_board=None):
    tcp=np.asarray(tcp_board,float);obj=np.asarray(T_board_object,float)
    region=GRASP_REGION if motion_profile is None else motion_profile['grasp_region']
    goal=grasp_point_board(obj,grasp_offset_board)
    local=obj[:3,:3].T@(goal-tcp)
    goal[2]=-region['tcp_height_target_cm']
    error=goal-tcp;height=-float(tcp[2])
    along_ready=abs(local[0])<=region['along_length_cm']
    across_ready=abs(local[1])<=region['across_width_cm']
    if grasp_offset_board is not None:
        if np.any(grasp_offset_board) and along_ready and not across_ready:
            # Preserve a usable contact position along the box while correcting its width.
            length_axis=obj[:2,0]
            axis_norm_sq=float(length_axis@length_axis)
            if axis_norm_sq>1e-8:
                error[:2]-=length_axis*float(length_axis@error[:2])/axis_norm_sq
    lateral_ready=along_ready and across_ready
    centered=abs(local[0])<=.7 and abs(local[1])<=.25
    ready=in_grasp_region(tcp,obj,region,grasp_offset_board=grasp_offset_board)
    if ready:return {'ready':True,'phase':'grasp_region','delta_board_cm':[0.,0.,0.]}
    if motion_profile is not None and motion_profile.get('version')!='legacy-v02':
        adaptive=motion_profile.get('adaptive_pitch_search')
        # This box is 2.5 cm tall. Above 3.5 cm, correct alignment at
        # the current height instead of repeatedly returning to the hover.
        alignment_floor=3.5 if adaptive else 4.2
        if adaptive and abs(local[0])<=region['along_length_cm'] and abs(local[1])<=.65:
            alignment_floor=3.3
        if not lateral_ready and height<alignment_floor:
            retreat_height=3.8 if adaptive else 4.5
            delta=np.array([0.,0.,-min(1.,retreat_height-height)])
            phase='raise_before_lateral_realign'
        else:
            delta=error.copy()
            if not lateral_ready:delta[2]=min(0.,delta[2])
            if adaptive:
                if lateral_ready and delta[2]>0:
                    delta[:2]=0.
                delta[2]=min(delta[2],1.0 if height>=3.2 else .5)
            length=np.linalg.norm(delta)
            if length>motion_profile['maximum_near_step_cm']:delta*=motion_profile['maximum_near_step_cm']/length
            phase='horizontal_alignment_at_clearance' if adaptive and not lateral_ready else 'joint_position_and_pitch_adjustment'
        return {'ready':False,'phase':phase,'delta_board_cm':delta.tolist()}
    # If contact-side clearance is uncertain, finish raising to a clear
    # alignment height; do not descend again halfway through that retreat.
    if not lateral_ready and height<4.2:
        delta=np.array([0.,0.,-min(1.,4.3-height)])
        phase='raise_to_alignment_clearance'
    elif height>=4.2 and not centered:
        delta=error.copy();delta[2]=0
        length=np.linalg.norm(delta[:2])
        if length>.6:delta[:2]*=.6/length
        phase='horizontal_alignment_before_descent'
    else:
        # Far approach goes to a clearance waypoint, then final fine descent.
        waypoint=3.8 if height>4.8 else 2.6
        down=height-waypoint
        maximum=2.5 if height>4.8 else .8
        delta=np.array([0.,0.,float(np.clip(down,-maximum,maximum))])
        phase='coarse_descent_to_clearance' if height>4.8 else 'final_fine_descent'
    return {'ready':False,'phase':phase,'delta_board_cm':delta.tolist()}
