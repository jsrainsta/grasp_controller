"""Offline nominal reachability study. Never emits executable arm requests.

Object center is used only as a tool-point proxy until actual TCP is calibrated.
Joint paths/collisions, jaw contact, and physical safety are not verified.
"""
import argparse
import json
import math
from pathlib import Path


def fk(positions, model):
    f = model['units_per_degree']
    zero = model['zero_positions']
    q3 = (positions['3'] - zero['3']) / f
    q4 = (positions['4'] - zero['4']) / f
    q5 = 90 - (positions['5'] - zero['5']) / f
    q6 = (positions['6'] - zero['6']) / f
    a, b, c, yaw = map(math.radians, (q5, q5+q4, q5+q4+q3, q6))
    r = model['upper_arm_cm']*math.cos(a) + model['forearm_cm']*math.cos(b) + model['tool_length_cm']*math.cos(c)
    z = model['base_height_cm'] + model['upper_arm_cm']*math.sin(a) + model['forearm_cm']*math.sin(b) + model['tool_length_cm']*math.sin(c)
    return [r*math.cos(yaw), r*math.sin(yaw), z], q5+q4+q3


def sin_bounds_degrees(lo, hi):
    values = [math.sin(math.radians(lo)), math.sin(math.radians(hi))]
    for k in range(math.ceil((lo-90)/180), math.floor((hi-90)/180)+1):
        values.append(math.sin(math.radians(90+180*k)))
    return min(values), max(values)


def vertical_enclosure(model, limits):
    """Conservative interval enclosure, not a sampled reachable workspace."""
    f, zero = model['units_per_degree'], model['zero_positions']
    angles = {str(i): ((limits[str(i)]['min']-zero[str(i)])/f,
                       (limits[str(i)]['max']-zero[str(i)])/f) for i in (3,4,5)}
    angles['5'] = (90-angles['5'][1], 90-angles['5'][0])
    intervals = [angles['5'], tuple(angles['5'][i]+angles['4'][i] for i in (0,1)),
                 tuple(angles['5'][i]+angles['4'][i]+angles['3'][i] for i in (0,1))]
    bounds = [model['base_height_cm']]*2
    for length, interval in zip((model['upper_arm_cm'], model['forearm_cm'], model['tool_length_cm']), intervals):
        s = sin_bounds_degrees(*interval)
        bounds = [bounds[i]+length*s[i] for i in (0,1)]
    return bounds


def ik(point, pitch, model):
    x,y,z = point
    r = math.hypot(x,y)
    alpha = math.radians(pitch)
    u = r-model['tool_length_cm']*math.cos(alpha)
    v = z-model['base_height_cm']-model['tool_length_cm']*math.sin(alpha)
    a,b = model['upper_arm_cm'],model['forearm_cm']
    cosine = (u*u+v*v-a*a-b*b)/(2*a*b)
    if not -1 <= cosine <= 1:
        return []
    result = []
    for sign in (-1,1):
        elbow = sign*math.acos(cosine)
        shoulder = math.atan2(v,u)-math.atan2(b*math.sin(elbow),a+b*math.cos(elbow))
        q5,q4 = math.degrees(shoulder),math.degrees(elbow)
        q3,q6 = pitch-q5-q4,math.degrees(math.atan2(y,x))
        if not (0 <= q5 <= 180 and -90 <= q4 <= 90 and -90 <= q3 <= 90):
            continue
        f,zero = model['units_per_degree'],model['zero_positions']
        positions = {'3': round(zero['3']+f*q3), '4': round(zero['4']+f*q4),
                     '5': round(zero['5']+f*(90-q5)), '6': round(zero['6']+f*q6)}
        rebuilt, actual_pitch = fk(positions,model)
        result.append({'illustrative_joint_positions_not_for_execution':positions,
                       'pitch_requested_deg':pitch, 'pitch_rounded_deg':actual_pitch,
                       'nominal_point_after_rounding_cm':rebuilt,
                       'rounding_error_cm':math.dist(point,rebuilt)})
    return result


def analyse(status, pose, location):
    model = status['data']['coordinate_settings']['model']
    limits = {str(row['id']):row for row in status['data']['servo_limits']}
    current = {str(row['id']):row['position'] for row in pose['data']['joint_positions']}
    current_fk,pitch = fk(current,model)
    stated = [pose['data'][k] for k in ('x_cm','y_cm','z_cm')]
    if math.dist(current_fk,stated)>1e-6:
        raise ValueError('Independent FK does not reproduce supplied model pose.')
    target = location['candidates'][0]['candidate_box_geometric_center_model_base_cm']
    z_bounds = vertical_enclosure(model,limits)
    phases = []
    for name,dz in (('preapproach_proxy',3),('contact_center_proxy',0),('lift_proxy',3)):
        point = [target[0],target[1],target[2]+dz]
        candidates = []
        for desired_pitch in range(-90,-39,5):
            for candidate in ik(point,desired_pitch,model):
                values=candidate['illustrative_joint_positions_not_for_execution']
                candidate['outside_configured_limits']=[int(i) for i,v in values.items()
                                                       if not limits[i]['min'] <= v <= limits[i]['max']]
                candidate['max_change_from_current_units']=max(abs(v-current[i]) for i,v in values.items())
                candidates.append(candidate)
        candidates.sort(key=lambda c:(len(c['outside_configured_limits']), abs(c['pitch_requested_deg']+90),c['max_change_from_current_units']))
        phases.append({'name':name, 'object_point_as_tool_proxy_cm':point,
                       'outside_vertical_enclosure':not z_bounds[0] <= point[2] <= z_bounds[1],
                       'nominal_geometric_candidate_count':len(candidates),
                       'candidate_count_within_current_target_limits':sum(not c['outside_configured_limits'] for c in candidates),
                       'illustrative_best_candidate':candidates[0] if candidates else None})
    return {'type':'offline_fixed_box_nominal_grasp_feasibility', 'version':1,
            'current_fk_matches_supplied_pose':True, 'current_fk_cm':current_fk, 'current_pitch_deg':pitch,
            'conservative_model_tool_z_enclosure_cm':z_bounds, 'configured_target_limits':list(limits.values()),
            'phases':phases, 'model_tool_point_equals_physical_contact_center_verified':False,
            'object_center_used_as_unverified_tool_point_proxy':True,
            'actual_contact_orientation_verified':False, 'collision_checked':False,
            'path_verified':False,'gripper_open_close_mapping_available':False,
            'robot_target':None,'executable_requests':[], 'for_motion':False,
            'conclusion':'Current configured target ranges exclude the proxy approach/contact points if outside the conservative Z enclosure. Nominal geometric solutions outside these ranges are not authorized physical ranges or executable poses.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('status','pose','location','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    data=[json.loads(getattr(args,k).read_text(encoding='utf-8')) for k in ('status','pose','location')]
    report=analyse(*data)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,allow_nan=False,indent=2)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
