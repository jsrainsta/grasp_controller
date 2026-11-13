"""Offline candidate hand-eye fit with planar branches and held-out views.

Uses nominal robot geometry. Results cannot enable motion or prove a grasp TCP.
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np

from check_grasp_board import ROOT
from sample_grasp_handeye import nominal_model


def transform(R, t):
    T = np.eye(4)
    T[:3,:3], T[:3,3] = R, np.asarray(t).reshape(3)
    return T


def robot_frame(positions, model, mount):
    p = {int(k):int(v) for k,v in positions.items()}
    angle = {j: (p[j]-model.zero_positions[str(j)])/model.units_per_degree for j in (3,4,5,6)}
    yaw = math.radians(angle[6])
    shoulder = math.radians(90-angle[5])
    elbow = math.radians(angle[4])
    pitch = shoulder+elbow+math.radians(angle[3])
    radial = model.upper_arm_cm*math.cos(shoulder)+model.forearm_cm*math.cos(shoulder+elbow)
    z = model.base_height_cm+model.upper_arm_cm*math.sin(shoulder)+model.forearm_cm*math.sin(shoulder+elbow)
    if mount == 'pitch_tool':
        radial += model.tool_length_cm*math.cos(pitch)
        z += model.tool_length_cm*math.sin(pitch)
        theta = pitch
    else:
        theta = shoulder+elbow
    cy, sy, ct, st = math.cos(yaw), math.sin(yaw), math.cos(theta), math.sin(theta)
    R = np.array([[cy*ct,-sy,-cy*st], [sy*ct,cy,-sy*st], [st,0,ct]])
    return transform(R, [radial*cy,radial*sy,z])


def preview_wrist_alignment(tool_rotation, box_width_axis, support_normal):
    """Nominal relative roll that aligns the closing axis in the support plane."""
    result={'for_motion':False,'requires_wrist_calibration':True}
    normal=np.asarray(support_normal,dtype=float)
    normal_length=np.linalg.norm(normal)
    if not np.isfinite(normal_length) or normal_length<1e-6:
        return {**result,'status':'roll_projection_underdetermined'}
    normal=normal/normal_length
    width=np.asarray(box_width_axis,dtype=float)
    width=width-normal*float(width@normal)
    width_length=np.linalg.norm(width)
    if not np.isfinite(width_length) or width_length<1e-6:
        return {**result,'status':'roll_projection_underdetermined'}
    width/=width_length
    along=np.cross(normal,width)
    closing,roll_tangent=tool_rotation[:,1],tool_rotation[:,2]
    lateral,tangent=float(along@closing),float(along@roll_tangent)
    if math.hypot(lateral,tangent)<1e-6:
        return {**result,'status':'roll_projection_underdetermined'}
    angle=math.atan2(-lateral,tangent)
    angle=(angle+math.pi/2)%math.pi-math.pi/2
    corrected=math.cos(angle)*closing+math.sin(angle)*roll_tangent
    return {**result,'status':'nominal_roll_preview',
            'relative_roll_deg':math.degrees(angle),
            'closing_axis_slope_to_support_deg':math.degrees(math.asin(np.clip(abs(corrected@normal),0,1)))}


def camera_candidates(board, K, D):
    profile = board['profile']
    obj = np.array([[(int(i)%6+1)*profile['square_length'],
                     (int(i)//6+1)*profile['square_length'],0] for i in board['corner_ids']],np.float64)
    img = np.array(board['corners_px'],np.float64)
    solved = cv2.solvePnPGeneric(obj,img,K,D,flags=cv2.SOLVEPNP_IPPE)
    if not solved[0]:
        raise ValueError('无法解算板相机位姿')
    result = []
    for rv,tv in zip(solved[1],solved[2]):
        R = cv2.Rodrigues(rv)[0]
        if np.min((R@obj.T+tv)[2]) <= 0:
            continue
        predicted = cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error = float(np.sqrt(np.mean(np.sum((predicted-img)**2,axis=1))))
        result.append((transform(R,tv),error))
    return sorted(result,key=lambda row:row[1])


def fit_AX_XB(G, C):
    pairs = [(np.linalg.inv(G[j])@G[i], C[j]@np.linalg.inv(C[i]))
             for i in range(len(G)) for j in range(i)]
    # Principal rotation vectors transform as a = R_X b.
    a = np.array([cv2.Rodrigues(A[:3,:3])[0].ravel() for A,B in pairs])
    b = np.array([cv2.Rodrigues(B[:3,:3])[0].ravel() for A,B in pairs])
    u,s,v = np.linalg.svd(a.T@b)
    R = u@np.diag([1,1,np.linalg.det(u@v)])@v
    lhs = np.concatenate([A[:3,:3]-np.eye(3) for A,B in pairs])
    rhs = np.concatenate([R@B[:3,3]-A[:3,3] for A,B in pairs])
    t,_,rank,sv = np.linalg.lstsq(lhs,rhs,rcond=None)
    if rank < 3 or sv[-1] < .01:
        raise ValueError('旋转激励不足，不能确定平移')
    X = transform(R,t)
    Ys = [g@X@c for g,c in zip(G,C)]
    u,_,v = np.linalg.svd(sum(y[:3,:3] for y in Ys))
    RY = u@np.diag([1,1,np.linalg.det(u@v)])@v
    Y = transform(RY,np.mean([y[:3,3] for y in Ys],axis=0))
    return X,Y,{'translation_condition':float(sv[0]/sv[-1]),'rotation_singular_values':s.tolist()}


def discrepancy(A, B):
    distance = float(np.linalg.norm(A[:3,3]-B[:3,3]))
    angle = float(np.linalg.norm(cv2.Rodrigues(A[:3,:3]@B[:3,:3].T)[0])*180/math.pi)
    return distance,angle


def fit_session(session_path):
    session = json.loads(session_path.read_text(encoding='utf-8'))
    observations = [json.loads(Path(r['path']).read_text(encoding='utf-8'))
                    for r in session['observations'] if r['accepted']]
    if len(observations) < 7:
        raise ValueError('至少需要7个合格视角，最后两张保留验证')
    intrinsic = json.loads((ROOT/'camera_calibration_results'/'diagnostic_20261026_58dc9618'/'intrinsics_candidate.json').read_text(encoding='utf-8'))
    K,D = np.array(intrinsic['camera_matrix_px']),np.array(intrinsic['distortion_coefficients'])
    C = [camera_candidates(o['board'],K,D) for o in observations]
    if any(not row for row in C):
        raise ValueError('存在无有效板位姿的样本')
    count = len(C)-2
    model = nominal_model()
    results = []
    for mount in ('pitch_tool','forearm'):
        G = [robot_frame(o['before']['joint_median'],model,mount) for o in observations]
        trials = []
        for branches in itertools.product(*[range(len(row)) for row in C[:count]]):
            cameras = [C[i][b][0] for i,b in enumerate(branches)]
            try:
                X,Y,condition = fit_AX_XB(G[:count],cameras)
            except (ValueError,np.linalg.LinAlgError):
                continue
            errors = [discrepancy(g@X@c,Y) for g,c in zip(G,cameras)]
            # Board must remain stationary; do not assume a fixed physical Z.
            score = float(np.mean([d*d+(math.radians(a)*15)**2 for d,a in errors]))
            score += .01*sum(C[i][b][1]**2 for i,b in enumerate(branches))/count
            trials.append((score,X,Y,branches,errors,condition))
        if not trials:
            continue
        trials.sort(key=lambda row:row[0])
        score,X,Y,branches,errors,condition = trials[0]
        held_out = []
        for i in range(count,len(C)):
            alternatives = [(discrepancy(G[i]@X@c,Y),b,rmse) for b,(c,rmse) in enumerate(C[i])]
            # Select a planar branch by predicted pose, never refit X/Y on validation.
            e,b,rmse = min(alternatives,key=lambda row:row[0][0]**2+(math.radians(row[0][1])*15)**2)
            held_out.append({'index':i,'branch':b,'position_error_cm':e[0],
                             'orientation_error_deg':e[1],'pnp_rmse_px':rmse})
        results.append({'mount_hypothesis':mount,'training_score':score,
            'T_mount_camera_cm':X.tolist(),'T_base_board_cm':Y.tolist(),
            'training_branches':list(branches),'training_errors_cm_deg':errors,
            'condition':condition,'held_out':held_out,
            'max_validation_position_cm':max(r['position_error_cm'] for r in held_out),
            'max_validation_orientation_deg':max(r['orientation_error_deg'] for r in held_out)})
    results.sort(key=lambda row:row['max_validation_position_cm'])
    report = {'type':'handeye_candidate_review','session':str(session_path.resolve()),
              'observation_count':len(observations),'training_count':count,'held_out_count':2,
              'camera_pnp_rmse_px':[[r[1] for r in row] for row in C],
              'candidates':results,'for_motion':False,'grasp_tcp_validated':False,
              'note':'Nominal robot geometry, manually measured print scale, unsynchronized static brackets; physical validation required.'}
    output = session_path.parent/'handeye_candidate_review.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in results:
        print(r['mount_hypothesis'],'验证位置误差cm：',round(r['max_validation_position_cm'],3),
              '角度deg：',round(r['max_validation_orientation_deg'],3))
    print('结果：',output)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    fit_session(parser.parse_args().session)
