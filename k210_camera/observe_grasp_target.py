"""Current-box observation using commissioned camera candidate; no arm writes."""
import argparse
from contextlib import nullcontext
from datetime import datetime
import hashlib
import json
from pathlib import Path
import uuid

import cv2
import numpy as np

from check_grasp_board import ROOT
from collect_grasp_reference import read_arm
from detect_box_outline import detect
from fit_grasp_handeye import robot_frame
from jpeg_integrity import check_jpeg
from sample_grasp_handeye import nominal_model
from vision_task import VisionTaskSession


def locate(im, ocr, joints, handeye, camera_override=None):
    intrinsic = json.loads((ROOT/'camera_calibration_results'/'diagnostic_20261026_58dc9618'/'intrinsics_candidate.json').read_text(encoding='utf-8'))
    K,D = np.array(intrinsic['camera_matrix_px']),np.array(intrinsic['distortion_coefficients'])
    camera = robot_frame(joints,nominal_model(),'pitch_tool') @ np.array(handeye['T_mount_camera_cm'])
    if camera_override is not None:
        camera=np.array(camera_override,dtype=np.float64)
    board = np.array(handeye['T_base_board_cm'])
    normal = board[:3,2]
    surface = board[:3,3]-normal*2.5
    outline = detect(im,ocr,'阿昔洛韦乳膏')
    if not outline['candidates']:
        from fit_known_box_outline import fit_outline
        fitted = fit_outline(im,ocr,K,D,camera,board)
        outline['metric_edge_fit'] = fitted
        if fitted and fitted['image_geometry_supported']:
            outline['candidates'] = [fitted]
    supported = []
    for candidate in outline['candidates']:
        pixel = np.array(candidate['corners_pixel'],np.float64)
        xy = cv2.undistortPoints(pixel.reshape(-1,1,2),K,D).reshape(-1,2)
        rays = (camera[:3,:3] @ np.column_stack((xy,np.ones(4))).T).T
        denominator = rays@normal
        if np.any(abs(denominator)<.05):
            continue
        distance = ((surface-camera[:3,3])@normal)/denominator
        if np.any(distance<=0):
            continue
        points = camera[:3,3]+rays*distance[:,None]
        edges = np.roll(points,-1,axis=0)-points
        lengths = np.linalg.norm(edges,axis=1)
        long = float((lengths[0]+lengths[2])/2)
        short = float((lengths[1]+lengths[3])/2)
        axis_long = edges[0]-edges[2]
        axis_long /= np.linalg.norm(axis_long)
        axis_short = edges[1]-edges[3]
        axis_short /= np.linalg.norm(axis_short)
        orthogonality = abs(float(axis_long@axis_short))
        dimensional_error = abs(long/10.5-1)+abs(short/3.4-1)
        row = {**candidate, 'top_corners_base_cm':points.tolist(),
            'top_center_base_cm':points.mean(0).tolist(),
            'geometric_center_base_cm':(points.mean(0)+normal*1.25).tolist(),
            'length_axis_base':axis_long.tolist(),'width_axis_base':axis_short.tolist(),
            'measured_length_cm':long,'measured_width_cm':short,
            'dimensional_error':dimensional_error,'orthogonality_error':orthogonality,
            'dimensions_independently_measured':candidate.get('method')!='known_blue_white_box_metric_rectangle_edge_fit',
            'dimension_note':'Metric edge fit imposes the known size; its dimensional residual is not independent validation.' if candidate.get('method')=='known_blue_white_box_metric_rectangle_edge_fit' else 'Ray-plane estimate; depends on plane and assumed height.',
            'geometry_supported':bool(abs(long/10.5-1)<=.18 and abs(short/3.4-1)<=.18 and orthogonality<.18)}
        supported.append(row)
    supported.sort(key=lambda row:row['dimensional_error']+row['orthogonality_error'])
    accepted = [r for r in supported if r['geometry_supported']]
    return {'outline':outline,'metric_candidates':supported,'selected':accepted[0] if accepted else None,
            'T_base_camera_cm':camera.tolist(),'table_down_normal_base':normal.tolist(),
            'assumed_box_dimensions_cm':[10.5,3.4,2.5],
            'for_motion':False,'grasp_tcp_validated':False}


def main(argv=None, camera=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-url',default='http://127.0.0.1:18000/v1/arm')
    parser.add_argument('--port',default='COM5')
    parser.add_argument('--calibration',type=Path,required=True)
    args = parser.parse_args(argv)
    cal = json.loads(args.calibration.read_text(encoding='utf-8'))
    best = next(r for r in cal['candidates'] if r['mount_hypothesis']=='pitch_tool')
    if best['max_validation_position_cm']>.3 or best['max_validation_orientation_deg']>1:
        raise RuntimeError('相机候选的保留验证未通过本阶段门槛')
    session_context = nullcontext(camera) if camera is not None else VisionTaskSession(
        args.port,'med_003',images=ROOT/'vision_grasp_images'/datetime.now().strftime('%Y%m%d'))
    with session_context as session:
        for attempt in range(2):
            before = read_arm(args.arm_url)
            if abs(before['joint_median']['2']-496)>3:
                raise RuntimeError('腕轴状态与本次安装标定基准不符')
            captured = session.capture('observe_'+uuid.uuid4().hex)
            if captured['status']!='captured':
                raise RuntimeError(str(captured))
            after = read_arm(args.arm_url)
            path = Path(captured['image_path'])
            integrity = check_jpeg(path)
            if integrity['valid']:
                break
            print('照片解码检查未通过；最多重拍一次，不发送机械臂动作。',flush=True)
        else:
            raise RuntimeError('两次图像均未通过完整性检查')
        stable = (all(abs(before['joint_median'][j]-after['joint_median'][j])<=2 for j in before['joint_median'])
                  and all(v<=6 for r in (before,after) for v in r['joint_span'].values())
                  and all(v<=2 for r in (before,after) for v in r['joint_mad'].values()))
        if not stable:
            raise RuntimeError('本次图像前后关节反馈不稳定，未用于定位')
        ocr = session.runner.run(path,ROOT/'ocr_results',known_box_focus=True)
    raw = path.read_bytes()
    im = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    report = {'type':'commissioning_box_observation','image':str(path),'sha256':hashlib.sha256(raw).hexdigest(),
              'before':before,'after':after,'ocr':ocr,'calibration':str(args.calibration.resolve()),
              'jpeg_integrity':integrity,'arm_motion_commands_sent':False}
    if ocr.get('target_selection',{}).get('status')=='target_found':
        report['geometry'] = locate(im,ocr,before['joint_median'],best)
    else:
        report['geometry'] = None
    directory=ROOT/'vision_grasp_results'
    directory.mkdir(exist_ok=True)
    stem=path.stem+'_'+uuid.uuid4().hex[:8]+'_commissioning'
    out=directory/(stem+'.json')
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    if report['geometry']:
        for row in report['geometry']['metric_candidates']:
            pts=np.rint(row['corners_pixel']).astype(np.int32)
            cv2.polylines(im,[pts],True,(0,255,0) if row['geometry_supported'] else (0,0,255),2)
    ok,encoded=cv2.imencode('.png',im)
    if ok:
        (directory/(stem+'.png')).write_bytes(encoded.tobytes())
    print('观察记录：',out)
    print('定位候选：',json.dumps(report.get('geometry',{}).get('selected') if report['geometry'] else None,ensure_ascii=False))


if __name__=='__main__':
    main()
