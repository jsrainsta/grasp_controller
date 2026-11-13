"""Capture or inspect the printed ChArUco reference; never controls the arm."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import uuid

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def inspect(path, profile_path):
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    raw = path.read_bytes()
    from jpeg_integrity import check_jpeg
    integrity = check_jpeg(path)
    if not integrity['valid']:
        raise ValueError('JPEG检查未通过：' + str(integrity))
    im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if im is None or im.shape != (480, 640, 3):
        raise ValueError('需要640×480的完整K210原图')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, profile['dictionary']))
    board = cv2.aruco.CharucoBoard((profile['squares_x'], profile['squares_y']),
        profile['square_length'], profile['marker_length'], dictionary,
        np.array(profile['marker_ids'], dtype=np.int32))
    board.setLegacyPattern(profile['legacy_pattern'])
    intrinsics_path=ROOT/'camera_calibration_results'/'diagnostic_20261026_58dc9618'/'intrinsics_candidate.json'
    intrinsics=json.loads(intrinsics_path.read_text(encoding='utf-8'))
    candidates = []
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    for minimum in (2,1):
        for scale in (1, 2):
            charuco_params=cv2.aruco.CharucoParameters()
            charuco_params.tryRefineMarkers=True
            charuco_params.minMarkers=minimum
            K=np.array(intrinsics['camera_matrix_px'],dtype=np.float64)
            K[:2]*=scale
            charuco_params.cameraMatrix=K
            charuco_params.distCoeffs=np.array(intrinsics['distortion_coefficients'],dtype=np.float64)
            detector_params=cv2.aruco.DetectorParameters()
            detector_params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
            detector=cv2.aruco.CharucoDetector(board,charuco_params,detector_params)
            sample = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            c, ci, mc, mi = detector.detectBoard(sample)
            candidates.append((0 if ci is None else len(ci), scale, minimum, c, ci, mc, mi))
    # More interpolated corners need not mean better geometry: minMarkers=1
    # can add false corners on the occluding box. Select by the same existing
    # geometric gate, not raw count, and retain the rejected alternatives.
    def rank(value):
        count,sc,mi,c,ci,mc,mid=value
        if count<4:return (False,0,0.,count)
        obj=board.getChessboardCorners()[ci.flatten(),:2]
        pts=c.reshape(-1,2)/sc
        H,mask=cv2.findHomography(obj,pts,cv2.RANSAC,1.5)
        if H is None or mask is None:return (False,0,0.,count)
        good=mask.ravel().astype(bool)
        residual=np.linalg.norm(cv2.perspectiveTransform(obj.reshape(-1,1,2),H).reshape(-1,2)-pts,axis=1)
        area=float(cv2.contourArea(cv2.convexHull(obj[good])))/((profile['squares_x']-2)*(profile['squares_y']-2)*profile['square_length']**2)
        valid=bool(good.sum()>=12 and good.mean()>=.9 and area>=.5 and residual[good].max()<=1.5)
        return (valid,int(good.sum()),float(good.mean()),count)
    variant_ranks=[rank(v) for v in candidates]
    chosen=max(range(len(candidates)),key=lambda i:variant_ranks[i])
    _, scale, minimum, corners, ids, marker_corners, marker_ids = candidates[chosen]
    if corners is not None:
        corners = corners / scale
    if marker_corners is not None:
        marker_corners = tuple(c / scale for c in marker_corners)
    count = 0 if ids is None else len(ids)
    geometry = {'consistent': False}
    if count >= 4:
        objects = board.getChessboardCorners()[ids.flatten(), :2]
        points = corners.reshape(-1, 2)
        H, mask = cv2.findHomography(objects, points, cv2.RANSAC, 1.5)
        if H is not None and mask is not None:
            good = mask.flatten().astype(bool)
            projected = cv2.perspectiveTransform(objects.reshape(-1, 1, 2), H).reshape(-1, 2)
            errors = np.linalg.norm(projected - points, axis=1)
            coverage = float(cv2.contourArea(cv2.convexHull(objects[good]))) / (
                (profile['squares_x']-2)*(profile['squares_y']-2)*profile['square_length']**2)
            maximum = float(errors[good].max())
            geometry = {'inliers': int(good.sum()), 'corner_hull_fraction': coverage,
                        'max_inlier_residual_px': maximum,
                        'consistent': bool(good.sum() >= 12 and good.mean() >= .9
                                           and coverage >= .5 and maximum <= 1.5),
                        'note': 'Image consistency only; does not prove paper flatness or physical scale.'}
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(im, marker_corners, marker_ids)
    if count:
        # Explicit overlay also works with OpenCV builds returning Nx2 corners.
        for point, corner_id in zip(corners.reshape(-1, 2), ids.reshape(-1)):
            x, y = np.rint(point).astype(int)
            cv2.circle(im, (x, y), 3, (0, 0, 255), 1)
            cv2.putText(im, str(int(corner_id)), (x+4, y-4),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 255), 1)
    report = {
        'type': 'grasp_board_visibility_check', 'image': str(path.resolve()),
        'sha256': hashlib.sha256(raw).hexdigest(), 'jpeg_integrity': integrity,
        'profile': profile, 'marker_count': 0 if marker_ids is None else len(marker_ids),
        'corner_count': count, 'all_corners_found': count == profile['expected_charuco_corners'],
        'detection_scale': scale, 'geometry_check': geometry,
        'variant_geometry_ranks':[{'scale':v[1],'min_markers':v[2],'rank':list(r)} for v,r in zip(candidates,variant_ranks)],
        'corner_interpolation_min_markers': minimum,
        'intrinsics_used_for_interpolation':str(intrinsics_path),
        'reference_candidate': geometry['consistent'],
        'corner_ids': [] if ids is None else ids.flatten().tolist(),
        'corners_px': [] if corners is None else corners.reshape(-1, 2).tolist(),
        'robot_transform_calibrated': False, 'motion_target_available': False,
        'note': 'Visibility check only; does not establish board-to-robot or grasp-point transforms.'}
    output = ROOT / 'calibration_checks'
    output.mkdir(exist_ok=True)
    stem = path.stem + '_' + uuid.uuid4().hex[:8] + '_charuco'
    (output / (stem + '.json')).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    ok, png = cv2.imencode('.png', im)
    if not ok:
        raise ValueError('检测图编码失败')
    (output / (stem + '.png')).write_bytes(png.tobytes())
    print('原图：', path)
    print('标记数量：', report['marker_count'], '角点：', count, '/ 24')
    print('ChArUco插值检查：', '角点几何通过，仍需独立求板位姿' if geometry['consistent'] else '插值角点不足；将另行检查真实编码标记，不据此单项判定定位失败')
    print('记录：', output / (stem + '.json'))
    print('检测图：', output / (stem + '.png'))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--capture', action='store_true')
    group.add_argument('--image', type=Path)
    parser.add_argument('--port')
    parser.add_argument('--profile', type=Path, default=ROOT/'calibration_boards'/'charuco_7x5_print96_v1.json')
    args = parser.parse_args()
    path = args.image
    if args.capture:
        from receive_photo import choose_port, receive_photos
        port = args.port or choose_port()
        path = receive_photos(port, 115200, ROOT/'calibration_images'/datetime.now().strftime('%Y%m%d'),
                              once=True, request_capture=True)
        if path is None:
            raise RuntimeError('未取得本次请求照片')
    inspect(Path(path), args.profile)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('本步失败：', error)
        raise SystemExit(1)
