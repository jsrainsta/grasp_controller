"""Check arm feedback and save a validated photo after restarting the Pi arm service."""

from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.error import URLError

from collect_grasp_reference import read_arm
from jpeg_integrity import check_jpeg
from verify_same_box_grasp import stable_loaded
from vision_task import VisionTaskSession
from types import SimpleNamespace


class StowedGraspVerifier:
    def __init__(self, config):
        settings = config["auto_stow"]
        self.config = config
        self.unit = settings.get("unit")
        if not isinstance(self.unit, str) or not re.fullmatch(r"[\w.-]+\.service", self.unit):
            raise ValueError("请填写树莓派机械臂的 systemd 用户服务名")

    def service(self, action):
        subprocess.run(["systemctl", "--user", action, "--", self.unit], check=True,
                       stdout=subprocess.DEVNULL, timeout=30)

    def verify_pose(self):
        state = read_arm(self.config["arm_url"])
        positions = [{str(joint["id"]): joint["position"]
                      for joint in pose["data"]["joint_positions"]} for pose in state["pose_samples"]]
        if not positions or any(row.keys() != set("123456") for row in positions):
            raise ValueError("复位后未取得完整关节反馈")
        if any(state["joint_span"][joint] > 3 for joint in "23456"):
            raise ValueError("复位后关节反馈尚未稳定")
        jaws = [row["1"] for row in positions]
        if not stable_loaded(jaws):
            raise ValueError("复位后夹爪反馈未通过")
        return state["joint_median"]

    def verify(self, run_id):
        self.service("restart")
        deadline = time.monotonic() + 25
        while True:
            try:
                before = self.verify_pose()
                break
            except URLError as error:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"重启后未能通过 HTTP 读取机械臂反馈：{error}") from error
                time.sleep(min(2.5, remaining))
            except (RuntimeError, ValueError) as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"复位反馈未通过：{error}") from error
                time.sleep(0.5)
        directory = Path(self.config["results_dir"]) / f"{run_id}_stow"
        runner = SimpleNamespace(target={"id": "med_003", "name": "阿昔洛韦乳膏"})
        with VisionTaskSession(self.config["camera_port"], "med_003", images=directory,
                               baudrate=self.config["camera_baudrate"], boot_wait=15,
                               runner=runner) as camera:
            capture = camera.capture(f"{run_id}-stow")
        photo = capture.get("image_path")
        if capture["status"] != "captured" or not photo or not check_jpeg(photo)["valid"]:
            raise ValueError(f"复位照片采集或校验失败，记录目录：{directory}")
        after = self.verify_pose()
        deltas = {joint: after[joint] - before[joint] for joint in before}
        limits = {joint: 3 if joint == "1" else 5 for joint in deltas}
        print(f"收臂拍照关节反馈 run_id={run_id} 拍照前={before} 拍照后={after} 差值={deltas} 容差={limits}",
              file=sys.stderr, flush=True)
        exceeded = {joint: delta for joint, delta in deltas.items() if abs(delta) > limits[joint]}
        if exceeded:
            raise ValueError(f"复位验收拍照期间关节发生移动，超限关节差值：{exceeded}")
        return {"photo": photo, "joint_positions": after}
