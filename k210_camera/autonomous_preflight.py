"""Read-only environment/arm snapshot. No POST, serial writes, or motion."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import sys
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


def get_json(base, path):
    request = Request(base.rstrip("/")+"/"+path, method="GET")
    try:
        with urlopen(request, timeout=5) as response:
            return json.load(response)
    except HTTPError as error:
        return {"ok":False,"http_status":error.code,
                "error_body":error.read().decode("utf-8",errors="replace")}


def collect(base, getter=get_json):
    parsed=urlsplit(base)
    if (parsed.scheme not in ("http","https") or not parsed.hostname
            or parsed.username or parsed.password or parsed.query or parsed.fragment):
        raise ValueError("需要不含凭据/查询参数的机械臂HTTP根地址")
    report={"type":"read_only_autonomous_grasp_preflight","version":1,
            "observed_at":datetime.now(timezone.utc).isoformat(),
            "arm_url":base,"environment":{"python":sys.version.split()[0],
                "system":platform.system(),"machine":platform.machine()},
            "requests":[],"errors":[],"hardware_commands_sent":False,
            "autonomous_grasp_ready":False,
            "readiness_note":"Connection diagnostics only; does not validate calibration, grasp point, or paths."}
    model=Path("/sys/firmware/devicetree/base/model")
    if platform.system()=="Linux":
        try:
            if model.exists():
                report["environment"]["pi_model"]=model.read_bytes().rstrip(b"\0").decode("utf-8",errors="replace")
            report["environment"]["serial_by_id"]=[str(p) for p in Path("/dev/serial/by-id").glob("*")]
        except OSError as error:
            report["errors"].append({"stage":"local_environment","message":str(error)})
    def read(path):
        report["requests"].append({"method":"GET","path":path})
        try:
            result=getter(base,path)
            report[path]=result
            return result
        except Exception as error:
            report["errors"].append({"stage":path,"message":str(error)})
            return None
    result=read("status")
    if not isinstance(result,dict) or result.get("ok") is not True or result.get("mode")!="serial":
        report["pose_query_skipped"]="status_unavailable_or_not_serial"
        return report
    state=result.get("data",{})
    if state.get("connection")!="connected":
        report["pose_query_skipped"]="arm_disconnected"
    elif state.get("active_task_id") is not None or state.get("upload_in_progress"):
        report["pose_query_skipped"]="active_task_or_upload_requires_review"
    else:
        read("pose?refresh=true")
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-url",required=True,help="例如 http://127.0.0.1:8000/v1/arm")
    parser.add_argument("--output",type=Path,help="可选：保存本次JSON")
    args=parser.parse_args()
    report=collect(args.arm_url)
    text=json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open("x",encoding="utf-8") as stream:
            stream.write(text)
    print(text)


if __name__=="__main__":
    main()
