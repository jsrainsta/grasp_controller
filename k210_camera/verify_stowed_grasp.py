"""Verify the transport pose and box after restarting the Pi arm service."""

from pathlib import Path
import re
import subprocess
import time

import cv2
import numpy as np

from collect_grasp_reference import read_arm
from jpeg_integrity import check_jpeg
from verify_same_box_grasp import stable_loaded
from vision_task import VisionTaskSession
from types import SimpleNamespace


def held_box_widths(image, roi):
    if image.shape[:2] != (480, 640):
        raise ValueError("复位验收需要 640×480 的 K210 原图")
    left, top, right, bottom = roi
    crop = image[top:bottom, left:right]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (85, 50, 38), (140, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((5, 11), dtype=np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(blue)
    if count == 1:
        return [0.0, 0.0]
    component = labels == (1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])))
    return [float(np.median(np.count_nonzero(band, axis=1)))
            for band in np.array_split(component, 2)]


class StowedGraspVerifier:
    def __init__(self, config):
        settings = config["auto_stow"]
        self.config = config
        self.unit = settings.get("unit")
        self.targets = settings.get("joint_targets")
        self.roi = settings.get("gripper_roi")
        if not isinstance(self.unit, str) or not re.fullmatch(r"[\w.-]+\.service", self.unit):
            raise ValueError("请填写树莓派机械臂的 systemd 用户服务名")
        if (not isinstance(self.targets, dict) or set(self.targets) != set("23456")
                or any(type(value) is not int or not 0 <= value <= 1000 for value in self.targets.values())):
            raise ValueError("请填写实测回位后 2～6 号关节位置")
        if (not isinstance(self.roi, list) or len(self.roi) != 4
                or any(type(value) is not int or value < 0 for value in self.roi)
                or not (0 <= self.roi[0] < self.roi[2] <= 640
                        and 0 <= self.roi[1] < self.roi[3] <= 480)):
            raise ValueError("请填写回位照片中夹爪持盒区域的四个像素坐标")

    def service(self, action):
        subprocess.run(["systemctl", "--user", action, "--", self.unit], check=True,
                       stdout=subprocess.DEVNULL, timeout=30)

    def verify_pose(self):
        state = read_arm(self.config["arm_url"])
        positions = [{str(joint["id"]): joint["position"]
                      for joint in pose["data"]["joint_positions"]} for pose in state["pose_samples"]]
        if (not positions or any(row.keys() != set("123456") for row in positions)
                or any(abs(row[joint] - target) > 10 for row in positions
                       for joint, target in self.targets.items())
                or any(state["joint_span"][joint] > 3 for joint in self.targets)):
            raise ValueError("机械臂未到达实测回位姿态")
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
            except (RuntimeError, ValueError) as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"复位反馈未通过：{error}") from error
                time.sleep(0.5)
        directory = Path(self.config["results_dir"]) / f"{run_id}_stow"
        runner = SimpleNamespace(target={"id": "med_003", "name": "阿昔洛韦乳膏"})
        with VisionTaskSession(self.config["camera_port"], "med_003", images=directory,
                               baudrate=self.config["camera_baudrate"], runner=runner) as camera:
            capture = camera.capture(f"{run_id}-stow")
        photo = capture.get("image_path")
        if capture["status"] != "captured" or not photo or not check_jpeg(photo)["valid"]:
            raise ValueError(f"复位照片采集或校验失败，记录目录：{directory}")
        after = self.verify_pose()
        if any(abs(after[joint] - before[joint]) > 3 for joint in before):
            raise ValueError("复位验收拍照期间关节发生移动")
        image = cv2.imdecode(np.fromfile(photo, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"无法解码复位照片：{photo}")
        widths = held_box_widths(image, self.roi)
        if min(widths) < 145:
            raise ValueError(f"未确认夹爪间的近景盒体，上下区宽度={widths}，照片：{photo}")
        return {"photo": photo, "near_box_widths_px": widths, "joint_positions": after}
