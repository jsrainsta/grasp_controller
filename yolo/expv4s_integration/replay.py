"""Replay immutable K210 images and joint feedback. All hardware ingress is replaced."""
import argparse
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ARM = HERE.parents[1] / "k210_camera"
sys.path.insert(0, str(ARM))
import check_grasp_board
import approach_grasp_candidate as hover
import guarded_visual_approach as near
from grasp_approach_policy import approach_decision
from grasp_visual_observer import GraspVisualObserver
from grasp_motion_profile import load_motion_profile
from observe_grasp_target import locate
from ocr_photo import OCRRunner
from register_current_scene import register
from jpeg_integrity import check_jpeg

ARCHIVE = ARM / "vision_grasp_results"
PROFILE = ARM / "grasp_motion_fixed_placement_trial.json"
FAR = ARM / "grasp_motion_far_placement_trial.json"
TCP = ARM / "calibration_boards/grasp_tcp_hover_candidate_20261105.json"


def load(path):
    return json.loads(Path(path).read_text(encoding="utf8"))


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf8")


def replay(raw, journal, folder):
    row = {"image":Path(raw["image"]).name,"frames":[]}
    src = Path(raw["image"])
    if not check_jpeg(src)["valid"] or raw["ocr"]["target_selection"]["status"] != "target_found":
        row["stop"] = "JPEG or OCR rejected"
        return row
    image = cv2.imdecode(np.fromfile(src,np.uint8),1)
    cal = next(c for c in load(raw["calibration"])["candidates"] if c["mount_hypothesis"]=="pitch_tool")
    raw = dict(raw, geometry=locate(image,raw["ocr"],raw["before"]["joint_median"],cal,image_path=src))
    source = folder / "overview.json"
    save(source, raw)
    row["initial_segmentation"] = raw["geometry"]["outline"].get("segmentation")
    try:
        intrinsic = Path("camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json")
        (folder/intrinsic.parent).mkdir(parents=True)
        shutil.copy(ARM/intrinsic,folder/intrinsic)
        with patch.object(check_grasp_board,"ROOT",folder):
            source = register(source,folder,motion=load_motion_profile(PROFILE))
    except ValueError as error:
        row["stop"] = str(error)
        return row
    target = load(source)["geometry"]["selected"]
    row.update(center=target["top_center_base_cm"], corners=target["corners_pixel"])
    args = list(map(str,[source,"--motion-profile",PROFILE,"--fallback-motion-profile",FAR,"--tcp-profile",TCP,"--high-start"]))
    def selected(path, motion, selection):
        row["plan"] = selection["mode"]
        row["profile"] = path
    try:
        with patch.object(hover,"read_arm",return_value=raw["after"]),patch.object(hover,"call",side_effect=AssertionError("network")):
            lift = hover.main(args,on_plan=selected)
        if lift:
            row["plan"] = "high_start_reobserve"
    except (ValueError,RuntimeError) as error:
        row["plan_error"] = str(error)
    if not journal:
        return row
    motion = load_motion_profile(Path(journal.get("motion_profile") or PROFILE))
    obs = GraspVisualObserver(source,TCP)
    handoff = journal.get("hover_handoff",{}).get("report")
    hover_data = load(handoff) if handoff else {}
    last_image = None
    if hover_data.get("captures"):
        frame = hover_data["captures"][-1]
        last_image = frame["image"]
        try:
            v = obs.estimate(last_image,frame["after"]["joint_median"])
            row["hover_frame"] = "accepted"
        except ValueError as error:
            last_image = None
            row["hover_frame"] = str(error)
    for pkt in journal["observations"]:
        path = pkt["capture"]["image_path"]
        fr = {"image":Path(path).name}
        row["frames"].append(fr)
        try:
            if path != last_image:
                v = obs.estimate(path,pkt["after"]["joint_median"])
            decision = approach_decision(v["tcp_board_cm"],v["T_board_object"],motion,
                                         grasp_offset_board=journal.get("grasp_offset_board_cm"))
            fr.update(phase=decision["phase"],delta=decision["delta_board_cm"],
                      segmentation=v["tracking"].get("segmentation"),
                      center=v["top_center_base_cm"],inliers=v["tracking"]["inlier_count"])
            last_image = path
        except ValueError as error:
            fr["error"] = str(error)
    # Run the actual guarded controller up to its first new command preview.
    packets = iter(journal["observations"])
    class Camera:
        def capture(self, _):
            packet = next(packets)
            return {"status":"captured", **packet["capture"]}
    camera = Camera()
    frames = journal["observations"]
    if not frames:
        return row
    reads = iter(v for p in frames for v in (p["before"],p["after"]))
    def read(_):
        return next(reads)
    args = ["--observation",str(source),"--motion-profile",str(journal.get("motion_profile") or PROFILE)]
    if hover_data:
        hand = folder/"hover.json"
        save(hand,dict(hover_data,source=str(source)))
        args.extend(["--hover-report",str(hand)])
    out = folder/"vision_grasp_results"
    out.mkdir()
    with patch.object(near,"ROOT",folder),patch.object(near,"read_arm",read),patch.object(near,"call",side_effect=AssertionError("network")):
        (folder/"calibration_boards").mkdir()
        shutil.copy(TCP,folder/"calibration_boards"/TCP.name)
        try:
            near.main(args,camera_session=camera)
        except (ValueError,RuntimeError,StopIteration) as error:
            row["first_guard_error"] = str(error) or "archive exhausted"
    reports = list(out.glob("visual_approach_*.json"))
    if reports:
        report = load(reports[0])
        row["first_guard"] = report["phase"]
        row["first_step"] = report.get("next_preview",{}).get("coordinate_plan",{}).get("targets")
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interpreter",required=True)
    p.add_argument("--day",default="20261112")
    p.add_argument("--seq",nargs="+",type=int,default=[2,4,9,17,21,33,45,51,62,79,94,104,39,76])
    args = p.parse_args()
    cv2.setNumThreads(2)
    runner = OCRRunner(target_id="med_003")
    reports = sorted(ARCHIVE.glob(f"visual_approach_{args.day}*.json"))
    by_source = {Path(j["source"]).name.replace("_registered.json",".json"):j for j in map(load,reports)}
    result = []
    with TemporaryDirectory() as temp:
        work = Path(temp)
        for seq in args.seq:
            paths = sorted(ARCHIVE.glob(f"photo_{args.day}_*_seq{seq}_*_commissioning.json"))
            if not paths:
                continue
            raw = load(paths[0])
            raw["ocr"] = runner.run(Path(raw["image"]),work/"ocr",known_box_focus=True)
            case = {}
            for variant in ("legacy","fusion"):
                folder = work/f"{seq}_{variant}"
                folder.mkdir()
                if variant=="fusion":
                    os.environ["GRASP_YOLO_PYTHON"]=args.interpreter
                else:
                    os.environ.pop("GRASP_YOLO_PYTHON",None)
                cv2.setRNGSeed(42)
                with redirect_stdout(io.StringIO()):
                    case[variant]=replay(raw,by_source.get(paths[0].name),folder)
            result.append(case)
            name = "replay_results.json" if args.day == "20261112" else f"replay_{args.day}.json"
            save(HERE/name,result)
            print(seq,[(k,v.get("plan"),v.get("first_guard"),str(v.get("stop",""))[:90]) for k,v in case.items()],flush=True)


if __name__=="__main__":
    main()
