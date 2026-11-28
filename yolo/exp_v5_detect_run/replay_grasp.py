"""Replay detector selection, board registration and nominal arm planning offline."""
import argparse
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

import cv2
import numpy as np

RUN = Path(__file__).resolve().parent
ARM = RUN.parents[1] / 'k210_camera'
sys.path.insert(0,str(ARM))
import approach_grasp_candidate as hover
import check_grasp_board
from grasp_medicine import MEDICINES, medicine
from observe_grasp_target import locate, select_detection
from register_current_scene import register

MOTION = ARM / 'grasp_motion_fixed_placement_trial.json'
FAR = ARM / 'grasp_motion_far_placement_trial.json'
TCP = ARM / 'calibration_boards/grasp_tcp_hover_candidate_20261105.json'
INTRINSICS = Path('camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json')


def load(path):
    return json.loads(Path(path).read_text(encoding='utf8'))


def replay(raw, target, folder):
    image = Path(raw['image'])
    report = {'image':image.name, 'name':target['name']}
    detection = select_detection(image,target)
    report['status'] = detection['status']
    if detection['selected'] is None:
        return report
    im = cv2.imdecode(np.fromfile(image,np.uint8),cv2.IMREAD_COLOR)
    calibration = next(row for row in load(raw['calibration'])['candidates']
                       if row['mount_hypothesis']=='pitch_tool')
    raw = dict(raw,ocr={},detection=detection)
    raw['geometry'] = locate(im,{},raw['before']['joint_median'],calibration,detection=detection)
    source = folder/'overview.json'
    source.write_text(json.dumps(raw,ensure_ascii=False),encoding='utf8')
    try:
        registered = register(source,motion=hover.load_motion_profile(MOTION))
        report['dimensions'] = load(registered)['geometry']['assumed_box_dimensions_cm']
        args = [registered,'--motion-profile',MOTION,'--fallback-motion-profile',FAR,
                '--tcp-profile',TCP,'--high-start']
        with patch.object(hover,'read_arm',return_value=raw['after']), patch.object(
                hover,'call',side_effect=AssertionError('hardware unavailable during replay')):
            result = hover.main(list(map(str,args)),on_plan=lambda _,__,row:report.update(plan=row['mode']))
        report['status'] = 'high_start_preview' if result else 'preview_passed'
    except (ValueError,RuntimeError) as error:
        report.update(status=type(error).__name__,error=str(error))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,default=RUN/'grasp_replay.json')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    reports = []
    with TemporaryDirectory() as temp:
        folder = Path(temp)
        (folder/INTRINSICS.parent).mkdir(parents=True)
        shutil.copyfile(ARM/INTRINSICS,folder/INTRINSICS)
        with patch.object(check_grasp_board,'ROOT',folder):
            for path in args.records:
                raw = load(path)
                if not Path(raw['image']).is_file():
                    continue
                for _,name,_ in MEDICINES:
                    with redirect_stdout(io.StringIO()):
                        report = replay(raw,medicine(name),folder)
                    reports.append(report)
                    print(report['image'],name,report['status'],flush=True)
                args.output.write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf8')


if __name__=='__main__':
    main()
