"""Replay the guarded controller with a local command comparator and archived feedback."""
import argparse
from contextlib import redirect_stdout
from datetime import datetime
import io
import os
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
from unittest.mock import patch

import cv2
import numpy as np
from replay import ARCHIVE, HERE, TCP, near, load, save, OCRRunner

def compare(source, journal, temp):
    hover=load(journal["hover_handoff"]["report"])
    hover_path=temp/"hover.json"
    save(hover_path, dict(hover,source=str(source)))
    frames=journal["observations"]
    reused="reuse_validation_read" in frames[0]
    now=datetime.fromisoformat(hover["captures"][-1]["after"]["received_at"]).timestamp()+(0 if reused else 3600)
    fresh=frames[1:] if reused else frames
    captures=iter(fresh)
    reads=([frames[0]["reuse_validation_read"]] if reused else [])+[r for f in fresh for r in (f["before"],f["after"])]
    reads=iter(reads)
    commands=iter(journal["commands"])
    confirmed=[]
    mismatch={}
    class Camera:
        def capture(self, _):
            return next(captures)["capture"]
    def recorded_reply(url, endpoint, request):
        expected=next(commands)
        def meaning(command):
            result={k:v for k,v in command.items() if k not in ("request_id","targets")}
            result["targets"]=sorted((v["id"],v["position"]) for v in command["targets"])
            return result
        if endpoint!=expected["endpoint"] or meaning(request)!=meaning(expected["request"]):
            mismatch.update(computed=meaning(request), recorded=meaning(expected["request"]))
            raise ValueError("computed_command_differs_from_archive")
        confirmed.append(meaning(request))
        return {"task":expected["task"]}
    (temp/"calibration_boards").mkdir(exist_ok=True)
    shutil.copy(TCP,temp/"calibration_boards"/TCP.name)
    (temp/"vision_grasp_results").mkdir()
    args=["--observation",str(source),"--arm-url","offline://archive","--hover-report",str(hover_path),
          "--motion-profile",journal["motion_profile"],"--execute"]
    reason=None
    with patch.object(near,"ROOT",temp),patch.object(near,"call",recorded_reply),patch.object(near,"read_arm",lambda _:next(reads)),patch.object(near.time,"time",return_value=now),patch.object(near.time,"sleep"):
        try:near.main(args,camera_session=Camera())
        except (ValueError,RuntimeError,StopIteration) as error:reason=str(error) or "archive_exhausted"
    result=load(next((temp/"vision_grasp_results").glob("visual_approach_*.json")))
    return dict(phase=result["phase"],reason=reason,first_difference=mismatch or None,matched_commands=len(confirmed),archived_commands=len(journal["commands"]))

def main():
    from replay import check_grasp_board, ARM, register, locate, PROFILE, load_motion_profile
    parser=argparse.ArgumentParser()
    parser.add_argument("--interpreter",required=True)
    args=parser.parse_args()
    out=[];runner=OCRRunner(target_id="med_003");cv2.setNumThreads(2)
    with TemporaryDirectory() as tmp:
        work=Path(tmp)
        for day,seq in [("20261112",33),("20261112",45),("20261112",51),("20261112",79),("20261112",104),("20261117",41),("20261117",119)]:
            rawpath=next(ARCHIVE.glob(f"photo_{day}_*_seq{seq}_*_commissioning.json"))
            raw=load(rawpath);src=Path(raw["image"])
            raw["ocr"]=runner.run(src,work/"ocr",known_box_focus=True)
            registered_name=rawpath.stem+"_registered.json"
            journal=next(j for j in map(load,sorted(ARCHIVE.glob(f"visual_approach_{day}*.json"))) if Path(j["source"]).name==registered_name)
            im=cv2.imdecode(np.fromfile(src,np.uint8),1)
            cal=next(c for c in load(raw["calibration"])["candidates"] if c["mount_hypothesis"]=="pitch_tool")
            case={"source":src.name}
            for mode in ("legacy","fusion"):
                temp=work/f"{day}_{seq}_{mode}";temp.mkdir()
                if mode=="fusion":os.environ["GRASP_YOLO_PYTHON"]=args.interpreter
                else:os.environ.pop("GRASP_YOLO_PYTHON",None)
                cv2.setRNGSeed(42)
                with redirect_stdout(io.StringIO()):
                    geometry=locate(im,raw["ocr"],raw["before"]["joint_median"],cal,image_path=src)
                    source=temp/"overview.json";save(source,dict(raw,geometry=geometry))
                    ic=Path("camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json")
                    (temp/ic.parent).mkdir(parents=True);shutil.copy(ARM/ic,temp/ic)
                    with patch.object(check_grasp_board,"ROOT",temp):
                        source=register(source,temp,motion=load_motion_profile(PROFILE))
                    case[mode]=compare(source,journal,temp)
            out.append(case);save(HERE/"command_replay.json",out)
            print(case,flush=True)

if __name__=="__main__":
    main()
