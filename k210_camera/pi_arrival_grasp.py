"""Run one stopped-arrival grasp from a JSON event on standard input."""

import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys

from grasp_controller import Coordinator, load_config


def handle_arrival(event, config_path):
    if not isinstance(event, dict) or event.get("target_id") != "med_003":
        raise ValueError("target_id 必须明确指定为 med_003")
    request_id = event.get("request_id")
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError("需要小车提供稳定的 request_id")
    config = load_config(config_path)
    port = config["camera_port"]
    if not port.startswith("/dev/") or not Path(port).exists():
        raise ValueError("请在树莓派配置中填写已经确认的 K210 设备路径")
    coordinator = Coordinator(config)
    if "auto_stow" in config:
        from verify_stowed_grasp import StowedGraspVerifier

        if config.get("close_only") is not True:
            raise ValueError("自动复位联调需要启用 close_only")
        verifier = StowedGraspVerifier(config)

        def run_and_verify(arrival, record):
            verifier.service("is-active")
            result = coordinator.run_pipeline(arrival, record)
            if result.get("closure_confirmed") is not True:
                raise RuntimeError("闭爪反馈未通过")
            result["stow_verification"] = verifier.verify(record["run_id"])
            result.update(status="stowed_verified", grasp_confirmed=True)
            return result

        coordinator.runner = run_and_verify
    return coordinator.handle(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    args = parser.parse_args()
    event = None
    try:
        event = json.load(sys.stdin)
        with redirect_stdout(sys.stderr):
            result = handle_arrival(event, args.config)
    except Exception as error:
        result = {"type": "autonomous_grasp_result", "version": 1,
                  "request_id": event.get("request_id") if isinstance(event, dict) else None,
                  "status": "rejected", "grasp_confirmed": False, "error": str(error)}
    result["travel_allowed"] = (
        result["status"] == "stowed_verified" and result.get("grasp_confirmed") is True
    )
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if result["travel_allowed"]:
        return 0
    if result["status"] == "held_verified" and result.get("grasp_confirmed") is True:
        return 0
    if result["status"] == "awaiting_stow_confirmation":
        return 3
    return 1 if result["status"] != "rejected" else 2


if __name__ == "__main__":
    sys.exit(main())
