"""Offline OCR-seeded box-face contour prototype; never commands hardware.

Known single rectangular package only. Candidates require human review and are
not calibrated robot targets. Rejects missing full-name evidence and border cuts.
"""
import argparse
import hashlib
import json
from pathlib import Path
import uuid

import cv2
import numpy as np


def detect(image, ocr, name):
    h, w = image.shape[:2]
    seeds = [np.array(row['polygon'], np.float32) for row in ocr.get('lines', [])
             if row.get('text') == name and row.get('confidence', 0) >= .8]
    result = {'status': 'needs_observation', 'reason': 'full_name_not_confirmed',
              'candidates': [], 'for_motion': False, 'robot_target': None,
              'method': 'ocr_seeded_edge_hull_prototype',
              'image_size': [w, h]}
    if not seeds:
        return result
    seed = max(seeds, key=cv2.contourArea)
    if seed.shape != (4, 2) or not np.isfinite(seed).all():
        raise ValueError('Invalid OCR polygon')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    candidates = []
    for low, high in ((20, 60), (30, 100), (50, 150)):
        edges = cv2.Canny(gray, low, high)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 1000:
                continue
            hull = cv2.convexHull(contour)
            quad = cv2.approxPolyDP(hull, .02*cv2.arcLength(hull, True), True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            quad = quad.reshape(4, 2).astype(np.float32)
            if (quad[:, 0].min() < 8 or quad[:, 1].min() < 8
                    or quad[:, 0].max() > w-9 or quad[:, 1].max() > h-9):
                continue
            area = abs(cv2.contourArea(quad))
            if not 2000 <= area <= h*w*.4:
                continue
            if any(cv2.pointPolygonTest(quad, tuple(map(float, p)), False) < 0 for p in seed):
                continue
            edges_xy = np.roll(quad, -1, axis=0)-quad
            lengths = np.linalg.norm(edges_xy, axis=1)
            ratio = (lengths[0]+lengths[2])/(lengths[1]+lengths[3])
            ratio = max(ratio, 1/ratio)
            if not 2 <= ratio <= 5:
                continue
            if area < 2*abs(cv2.contourArea(seed)):
                continue
            if any(np.linalg.norm(quad.mean(0)-np.array(c['center_pixel'])) < 8
                   and abs(area/c['area_px2']-1) < .15 for c in candidates):
                continue
            # Start with a long physical edge, preserving the cyclic contour.
            start = int(np.argmax(lengths))
            quad = np.roll(quad, -start, axis=0)
            candidates.append({'corners_pixel': quad.tolist(),
                               'center_pixel': quad.mean(0).tolist(),
                               'area_px2': float(area),
                               'projected_aspect_ratio': float(ratio),
                               'canny_thresholds': [low, high]})
    candidates.sort(key=lambda c: c['area_px2'])
    result['candidates'] = candidates
    if candidates:
        result.update(status='outline_candidates_for_review',
                      reason='complete_quad_with_full_name_inside')
    else:
        result['reason'] = 'no_complete_supported_quad'
    return result


def main():
    parser = argparse.ArgumentParser(description='离线药盒轮廓候选，非机械臂目标')
    parser.add_argument('image', type=Path)
    parser.add_argument('--ocr', type=Path, required=True)
    parser.add_argument('--name', default='阿昔洛韦乳膏')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent/'vision_grasp_results')
    args = parser.parse_args()
    raw = args.image.read_bytes()
    im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        raise ValueError('Cannot decode image')
    ocr = json.loads(args.ocr.read_text(encoding='utf-8'))
    if Path(ocr['image_path']).resolve() != args.image.resolve():
        raise ValueError('OCR belongs to another image')
    if (ocr['image_width'], ocr['image_height']) != (im.shape[1], im.shape[0]):
        raise ValueError('OCR image dimensions differ')
    report = detect(im, ocr, args.name)
    report.update(image_path=str(args.image.resolve()), image_sha256=hashlib.sha256(raw).hexdigest(),
                  ocr_path=str(args.ocr.resolve()), name=args.name,
                  limitations=['Single known rectangular box; OCR-seeded heuristic.',
                               'Not a learned general detector or validated instance segmentation.',
                               'Corner completeness and geometry need human review.',
                               'Pixel center is not a calibrated grasp point.'])
    overlay = im.copy()
    for i, c in enumerate(report['candidates']):
        pts = np.rint(c['corners_pixel']).astype(np.int32)
        cv2.polylines(overlay, [pts], True, (0, 220, 255), 2)
        center = tuple(np.rint(c['center_pixel']).astype(int))
        cv2.drawMarker(overlay, center, (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
        cv2.putText(overlay, str(i), center, cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
    cv2.putText(overlay, 'OUTLINE REVIEW ONLY - NO MOTION', (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 255), 1)
    args.output.mkdir(parents=True, exist_ok=True)
    stem = args.image.stem+'_'+uuid.uuid4().hex[:8]+'_outline'
    target = args.output/(stem+'.json')
    target.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
    ok, png = cv2.imencode('.png', overlay)
    if not ok:
        raise ValueError('Cannot encode overlay')
    png.tofile(args.output/(stem+'.png'))
    print(report['status'], report['reason'], '候选数：', len(report['candidates']))
    print(target.resolve())
    print((args.output/(stem+'.png')).resolve())


if __name__ == '__main__':
    main()
