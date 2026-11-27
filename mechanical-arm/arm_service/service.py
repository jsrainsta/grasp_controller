"""串口独占、任务状态和固件能力约束。HTTP 层不直接操作串口。"""

from __future__ import annotations

import copy
import json
import logging
import math
import statistics
import threading
import time
import uuid
from collections import OrderedDict
from datetime import datetime, timezone

from .config import Settings
from .coordinate_control import CorrectionError, evaluate_feedback, plan_correction
from .errors import ArmError
from .kinematics import KinematicsError, KinematicsModel
from .protocol import (Command, RESET_POSITIONS, decode_offsets,
                       decode_positions, decode_version, encode_packet,
                       group_frame_payload, group_run_payload, move_payload, offset_payload)
from .transport import MockTransport, SerialTransport, TransportError
from .validation import gripper_parameters

LOG = logging.getLogger(__name__)


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class ArmService:
    def __init__(self, settings: Settings | None = None, transport=None):
        self.settings = settings or Settings()
        self._kinematics = KinematicsModel.from_config(self.settings.kinematics)
        self.transport = transport if transport is not None else (
            MockTransport() if self.settings.mode == "mock" else
            SerialTransport(self.settings.serial_port, self.settings.baudrate,
                            self.settings.write_timeout_s, self.settings.inter_command_delay_s))
        self._lock = threading.RLock()
        self._request_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._cancel_upload = threading.Event()
        self._thread = None
        self._connected = False
        self._last_error = None
        self._device = None
        self._positions = None
        self._position_time = None
        self._position_mono = None
        self._tasks = OrderedDict()
        self._active_id = None
        self._requests = OrderedDict()
        self._groups = {}
        self._uploading = False
        self._query_not_before = 0.0
        self._limits_require_recheck = False
        self._motion_state = "unobserved"

    def start(self, monitor: bool = True) -> None:
        """显式启动时才连接；启动只查询，不发送运动或复位。"""
        with self._lock:
            if self._thread is not None:
                return
            self._shutdown.clear()
            try:
                self._connect()
            except ArmError as error:
                LOG.warning("设备初始化失败：%s", error.message)
            if monitor and not self._shutdown.is_set():
                self._thread = threading.Thread(target=self._monitor, name="arm-feedback", daemon=True)
                self._thread.start()

    def close(self) -> None:
        self._shutdown.set()
        self._cancel_upload.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.settings.query_timeout_s + self.settings.write_timeout_s + 2)
        with self._lock:
            if self._active_id is not None:
                self._unknown_task("服务已关闭，运动结果未确认")
            self.transport.close()
            self._connected = False
            self._thread = None

    def _connect(self) -> None:
        self._connected = False
        self._device = None
        self._positions = None
        self._position_time = self._position_mono = None
        stage = "open"
        try:
            LOG.info("连接初始化：打开通信通道，模式 %s", self.settings.mode)
            self.transport.open()
            if self.settings.mode == "serial":
                stage = "startup_wait"
                LOG.info("串口已打开，等待设备初始化 %.3f 秒", self.settings.startup_delay_s)
                if self._shutdown.wait(self.settings.startup_delay_s):
                    raise ArmError("connection_cancelled", "连接初始化已取消", 503)
            stage = "version"
            LOG.info("连接初始化：查询固件信息（0x01）")
            packet = self.transport.exchange(encode_packet(Command.VERSION), int(Command.VERSION), self.settings.query_timeout_s)
            device = decode_version(packet.payload)
            if device["arm_type"] != "bus":
                self.transport.close()
                raise ArmError("unsupported_arm_type", "此版本适用于总线款舵机，设备返回 PWM 类型", 503)
            self._device = {**device, "observed_at": now(), "source": self.settings.mode}
            LOG.info("固件信息读取成功：类型 %s，版本 %s", device["arm_type"], device["firmware_version"])
            if self._shutdown.is_set():
                raise ArmError("connection_cancelled", "连接初始化已取消", 503)
            stage = "positions"
            LOG.info("连接初始化：查询六个舵机位置（0x0D）")
            self._connected = True
            self._read_positions()
            self._last_error = None
            LOG.info("连接初始化完成，六个舵机位置已读取")
        except TransportError as error:
            LOG.warning("连接初始化失败，阶段=%s：%s", stage, error.message)
            raise self._communication_error(error)
        except ValueError as error:
            LOG.warning("连接初始化失败，阶段=%s：%s", stage, error)
            raise self._invalid_reply(error)
        except ArmError as error:
            LOG.warning("连接初始化失败，阶段=%s：%s", stage, error.message)
            self._connected = False
            self.transport.close()
            self._last_error = error.as_dict()
            raise

    def _invalid_reply(self, error: Exception, code: str = "invalid_reply", status: int = 502) -> ArmError:
        self._connected = False
        self.transport.close()
        self._unknown_task("设备回复无效，此前动作结果未确认")
        result = ArmError(code, str(error), status, self._active_id is not None)
        self._last_error = result.as_dict()
        return result

    def _communication_error(self, error: TransportError) -> ArmError:
        self._connected = False
        self.transport.close()
        self._unknown_task("通信失败，无法确认此前运动结果")
        result = ArmError(error.code, error.message,
                          504 if error.code == "communication_timeout" else 503,
                          error.outcome_unknown or self._active_id is not None)
        self._last_error = result.as_dict()
        LOG.warning("通信失败：%s", error.message)
        return result

    def _require_connected(self) -> None:
        if not self._connected or not self.transport.is_open:
            raise ArmError("not_connected", "设备未连接；检查配置或调用重连接口", 503)

    def _exchange(self, command: Command, payload: bytes = b"", reply: bool = False):
        self._require_connected()
        try:
            return self.transport.exchange(encode_packet(command, payload),
                                           int(command) if reply else None,
                                           self.settings.query_timeout_s)
        except TransportError as error:
            raise self._communication_error(error)

    def _active(self):
        return self._tasks.get(self._active_id)

    def _idle(self) -> None:
        if self._active_id is not None or self._uploading:
            raise ArmError("busy", "有尚未结束的任务；请查询状态，未知任务可在现场确认静止后显式释放")

    def _query_allowed(self) -> None:
        task = self._active()
        if self._uploading or (task is not None and task["type"] == "group"):
            raise ArmError("query_blocked", "当前任务期间仅支持缓存查询与停止；新串口查询可能中断固件动作")

    def _write_allowed(self, feature: str | None = None) -> None:
        self._require_connected()
        if self.settings.mode == "serial" and not self.settings.allow_hardware_writes:
            raise ArmError("hardware_writes_disabled", "当前串口配置只允许读取；尚未启用实物写命令", 403)
        if feature and feature not in self.settings.enabled_features:
            raise ArmError("feature_disabled", f"未启用 {feature} 功能", 403)

    def _check_targets(self, targets: dict[int, int]) -> None:
        if self._limits_require_recheck:
            raise ArmError("limits_require_recheck", "偏差修改后需重新核验关节范围、更新配置并重启服务", 403)
        limits = self.settings.limits()
        for servo_id, position in targets.items():
            if servo_id not in limits:
                raise ArmError("limits_missing", f"尚未配置舵机 {servo_id} 的实际有效范围", 403)
            low, high = limits[servo_id]
            if not low <= position <= high:
                raise ArmError("position_out_of_range", f"舵机 {servo_id} 目标必须在 {low}～{high} 范围内", 422)

    def _new_task(self, kind: str, request_id: str, targets: dict | None = None) -> dict:
        task_id = uuid.uuid4().hex
        task = {"task_id": task_id, "request_id": request_id, "type": kind, "status": "sent",
                "created_at": now(), "updated_at": now(), "targets": targets or {},
                "feedback_confirmed": False, "physically_stopped": None,
                "consecutive_samples": 0, "message": "命令已发送，尚未确认到位"}
        task["_started_mono"] = time.monotonic()
        if kind in {"move", "reset"}:
            task["acceptance"] = {"position_tolerance": self.settings.position_tolerance,
                                  "required_samples": self.settings.reached_samples}
        self._tasks[task_id] = task
        self._active_id = task_id
        self._motion_state = "command_sent"
        while len(self._tasks) > 256:
            self._tasks.popitem(last=False)
        return task

    @staticmethod
    def _public_task(task: dict) -> dict:
        return copy.deepcopy({key: value for key, value in task.items() if not key.startswith("_")})

    def _unknown_task(self, reason: str, code: str = "outcome_unknown") -> None:
        task = self._active()
        if task:
            task.update(status="outcome_unknown", message=reason, updated_at=now())
            if task["type"] == "coordinate":
                task["correction"].update(state="stopped", reason=code)
            elif task["type"] == "gripper_close":
                task.update(observation_state="stopped", failure_reason={"code": code, "message": reason})
        self._motion_state = "outcome_unknown"

    def _evaluate_coordinate(self, task: dict, positions: dict, observed_at: str | None = None) -> dict:
        acceptance = task["coordinate_plan"]["acceptance"]
        return evaluate_feedback(
            self._kinematics, positions, task["coordinate_plan"],
            joint_tolerance=acceptance["joint_position_tolerance"],
            position_tolerance_cm=acceptance["effective_position_tolerance_cm"],
            pitch_tolerance_deg=acceptance["pitch_tolerance_deg"], observed_at=observed_at,
        )

    def _record_coordinate_feedback(self, task: dict, positions: dict, allow_acceptance: bool = True) -> None:
        try:
            feedback = self._evaluate_coordinate(task, positions, self._position_time)
        except (KinematicsError, ValueError, TypeError, OverflowError) as error:
            task["coordinate_feedback"] = {
                "valid": False, "observed_at": self._position_time,
                "joint_positions": copy.deepcopy(positions),
                "error": {"code": "coordinate_feedback_invalid", "message": str(error)},
                "last_valid": copy.deepcopy(task.get("coordinate_feedback", {}).get("last_valid")
                                            if task.get("coordinate_feedback", {}).get("valid") is False
                                            else task.get("coordinate_feedback")),
            }
            task.update(joint_feedback_confirmed=False, model_feedback_confirmed=False,
                        consecutive_samples=0, joint_consecutive_samples=0, model_consecutive_samples=0)
            self._unknown_task("坐标反馈无法解算；已停止后续修正，执行结果未知", "coordinate_feedback_invalid")
            self._last_error = ArmError("coordinate_feedback_invalid", str(error), 502, True).as_dict()
            return
        task["coordinate_feedback"] = feedback
        if not allow_acceptance:
            task["updated_at"] = now()
            return
        task["joint_consecutive_samples"] = task["joint_consecutive_samples"] + 1 if feedback["joint_within_tolerance"] else 0
        task["model_consecutive_samples"] = task["model_consecutive_samples"] + 1 if feedback["model_within_tolerance"] else 0
        task["consecutive_samples"] = task["consecutive_samples"] + 1 if feedback["within_tolerance"] else 0
        required = task["coordinate_plan"]["acceptance"]["required_samples"]
        task["joint_feedback_confirmed"] = task["joint_consecutive_samples"] >= required
        task["model_feedback_confirmed"] = task["model_consecutive_samples"] >= required
        task["updated_at"] = now()
        samples = task["_coordinate_samples"]
        sample = (self._position_mono, {i: positions[i] for i in range(3, 7)})
        if samples and self._position_mono - samples[-1][0] + 1e-9 < self.settings.poll_interval_s:
            # 快速重复 GET 不增加稳定样本；其间若发现跳变，稳定观察重新开始。
            if any(max([old[1][i] for old in samples] + [positions[i]])
                   - min([old[1][i] for old in samples] + [positions[i]])
                   > task["correction"]["stability_tolerance"] for i in range(3, 7)):
                samples[:] = [sample]
        else:
            samples.append(sample)
            del samples[:-task["correction"]["stable_samples"]]
        stable_positions, _ = self._coordinate_stable_window(task)
        stable_within = False
        if stable_positions is not None:
            try:
                stable_feedback = self._evaluate_coordinate(task, stable_positions, self._position_time)
                task["correction"]["stable_feedback"] = copy.deepcopy(stable_feedback)
                stable_within = stable_feedback["within_tolerance"]
            except (KinematicsError, ValueError, TypeError, OverflowError) as error:
                self._unknown_task("稳定坐标反馈无法解算；已停止后续修正", "coordinate_feedback_invalid")
                self._last_error = ArmError("coordinate_feedback_invalid", str(error), 502, True).as_dict()
                return
        if (task["consecutive_samples"] >= required and stable_within and not self._shutdown.is_set()
                and not self._motion_timed_out(task)):
            task.update(status="reached", feedback_confirmed=True,
                        message="关节位置与反馈模型位置、俯仰同时连续达标；不代表末端独立测量")
            task["correction"].update(state="complete", reason="acceptance_reached")
            self._active_id = None
            self._motion_state = "target_feedback_reached"

    def _coordinate_stable_window(self, task: dict) -> tuple[dict | None, str]:
        correction = task["correction"]
        if time.monotonic() - task["_last_command_mono"] < correction["settle_s"]:
            return None, "settle_time"
        samples = task["_coordinate_samples"]
        needed = correction["stable_samples"]
        minimum_span = self.settings.poll_interval_s * (needed - 1)
        if len(samples) < needed or samples[-1][0] - samples[0][0] + 1e-9 < minimum_span:
            return None, "insufficient_stable_samples"
        if any(max(sample[1][i] for sample in samples) - min(sample[1][i] for sample in samples)
               > correction["stability_tolerance"] for i in range(3, 7)):
            return None, "feedback_unstable"
        return {i: int(round(statistics.median(sample[1][i] for sample in samples))) for i in range(3, 7)}, "stable"

    def _read_positions(self) -> dict:
        task = self._active()
        if task and task["type"] in {"move", "reset", "coordinate", "gripper_close"} and task["status"] == "sent":
            self._motion_timed_out(task)
        remaining = self._query_not_before - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        packet = self._exchange(Command.POSITIONS_READ, reply=True)
        try:
            positions = decode_positions(packet.payload)
        except ValueError as error:
            raise self._invalid_reply(error)
        self._positions = positions
        self._position_time = now()
        self._position_mono = time.monotonic()
        task = self._active()
        if task and task["type"] in {"move", "reset", "coordinate", "gripper_close"} and task["status"] == "sent":
            self._motion_timed_out(task)
        if task and task["type"] == "coordinate" and task["status"] in {"sent", "outcome_unknown"}:
            self._record_coordinate_feedback(task, positions,
                                             allow_acceptance=task["status"] == "sent" and not self._shutdown.is_set())
        elif task and task["type"] in {"move", "reset"} and task["status"] in {"sent", "outcome_unknown"}:
            self._record_position_feedback(task, positions)
        elif task and task["type"] == "gripper_close" and task["status"] in {"sent", "outcome_unknown"}:
            self._record_gripper_feedback(task, positions[1])
        return self._position_result(fresh=True)

    def _record_position_feedback(self, task: dict, positions: dict) -> None:
        acceptance = task["acceptance"]
        errors = {int(key): positions[int(key)] - value for key, value in task["targets"].items()}
        within = all(abs(error) <= acceptance["position_tolerance"] for error in errors.values())
        task["position_feedback"] = {
            "observed_at": self._position_time, "joint_positions": copy.deepcopy(positions),
            "target_errors": errors, "within_tolerance": within, **acceptance,
        }
        task["updated_at"] = now()
        # 晚到反馈只补充诊断，不能把此前结果未知的任务追认为成功。
        if task["status"] != "sent" or self._shutdown.is_set():
            return
        task["consecutive_samples"] = task["consecutive_samples"] + 1 if within else 0
        if task["consecutive_samples"] >= acceptance["required_samples"]:
            task.update(status="reached", feedback_confirmed=True,
                        message="连续位置反馈处于目标容差内；不代表夹持成功或末端 XYZ 精度保证")
            self._active_id = None
            self._motion_state = "target_feedback_reached"

    @staticmethod
    def _gripper_region(task: dict, position: int) -> str:
        acceptance = task["acceptance"]
        if abs(position - task["targets"][1]) <= acceptance["goal_tolerance"]:
            return "close_target_reached"
        region = acceptance["expected_contact_range"]
        if region["min"] <= position <= region["max"]:
            return "stable_in_expected_contact_region"
        return "outside_expected_regions"

    def _record_gripper_feedback(self, task: dict, position: int) -> None:
        """只观察，绝不因受阻而追加命令；稳定位置不是接触或夹紧测量。"""
        acceptance = task["acceptance"]
        target, start = task["targets"][1], task["start_positions"][1]
        region = self._gripper_region(task, position)
        feedback = {
            "observed_at": self._position_time, "position": position, "close_target": target,
            "target_error": position - target, "closing_delta": position - start,
            "in_expected_contact_region": region == "stable_in_expected_contact_region",
            "position_goal_reached": region == "close_target_reached",
            "sample_count": 0, "span_s": 0.0, "spread": 0, "stable": False,
        }
        task["gripper_feedback"] = feedback
        task["updated_at"] = now()
        if task["status"] != "sent" or self._shutdown.is_set():
            return
        if self._position_mono - task["_last_command_mono"] < acceptance["settle_s"]:
            return
        samples = task["_gripper_samples"]
        # 即使快速 GET 不增加采样数，其间发现越区或跳变也使稳定观察重新开始。
        changed_region = samples and task["_gripper_sample_region"] != region
        minimum = min(task["_gripper_min_position"], position) if samples else position
        maximum = max(task["_gripper_max_position"], position) if samples else position
        if changed_region or maximum - minimum > acceptance["stability_tolerance"]:
            samples.clear()
            minimum = maximum = position
        task["_gripper_min_position"], task["_gripper_max_position"] = minimum, maximum
        task["_gripper_sample_region"] = region
        if not samples or self._position_mono - samples[-1][0] + 1e-9 >= acceptance["poll_interval_s"]:
            samples.append((self._position_mono, position))
        span = samples[-1][0] - samples[0][0]
        spread = maximum - minimum
        required_span = max(acceptance["stable_s"],
                            acceptance["poll_interval_s"] * (acceptance["stable_samples"] - 1))
        stable = len(samples) >= acceptance["stable_samples"] and span + 1e-9 >= required_span
        feedback.update(sample_count=len(samples), span_s=round(span, 6), spread=spread, stable=stable)
        task["consecutive_samples"] = len(samples)
        if not stable:
            return
        median_position = int(round(statistics.median(p for _, p in samples)))
        task["stable_feedback"] = {
            "position": median_position, "observed_at": self._position_time,
            "sample_count": len(samples), "span_s": round(span, 6), "spread": spread,
            "min_position": minimum, "max_position": maximum,
            "closing_delta": median_position - start, "target_error": median_position - target,
            "position_goal_reached": region == "close_target_reached",
        }
        if self._motion_timed_out(task) or self._shutdown.is_set():
            return
        if min(p for _, p in samples) - start < acceptance["min_closing_delta"]:
            self._unknown_task("未观察到足够的实际闭合位移；未自动追加命令", "gripper_no_closing_progress")
            return
        if region == "outside_expected_regions":
            self._unknown_task("夹爪稳定位置不在预计接触区或目标到位区；未自动加压或松爪",
                               "gripper_unexpected_stall")
            return
        task.update(status="closure_observed", observation_state="complete", feedback_confirmed=True,
                    completion_reason=region, position_goal_reached=region == "close_target_reached",
                    message="闭合位置观察完成；未确认接触、夹持力或夹稳，原闭合目标未撤销")
        self._active_id = None
        self._motion_state = "gripper_closure_observed"

    def _position_result(self, fresh: bool = False) -> dict:
        if self._positions is None:
            raise ArmError("positions_unavailable", "尚无可用的位置反馈", 503)
        return {"positions": [{"id": key, "position": value} for key, value in sorted(self._positions.items())],
                "source": self.settings.mode, "observed_at": self._position_time,
                "age_ms": round((time.monotonic() - self._position_mono) * 1000), "fresh": fresh}

    def _monitor(self) -> None:
        while not self._shutdown.wait(self.settings.poll_interval_s):
            self._monitor_once()

    def _motion_timed_out(self, task: dict) -> bool:
        gripper = task["type"] == "gripper_close"
        timeout = task["acceptance"]["timeout_s"] if gripper else self.settings.motion_timeout_s
        if time.monotonic() - task["_started_mono"] < timeout:
            return False
        if gripper:
            self._unknown_task("夹爪闭合观察超时；原目标未撤销，未自动重发、停止或松爪",
                               "gripper_observation_timeout")
        else:
            self._unknown_task("到位反馈超时；未自动返程或停止，执行结果未知", "feedback_timeout")
        return True

    def _monitor_once(self) -> None:
        """后台的一轮处理；HTTP 查询不会调用本方法或提交修正。"""
        with self._lock:
            task = self._active()
            if (self._shutdown.is_set() or not task
                    or task["type"] not in {"move", "reset", "coordinate", "gripper_close"} or task["status"] != "sent"):
                return
            if self._motion_timed_out(task):
                return
            try:
                self._read_positions()
                if (self._active() is task and task["status"] == "sent" and task["type"] == "coordinate"
                        and not self._motion_timed_out(task)):
                    self._monitor_coordinate_correction(task)
            except ArmError as error:
                self._last_error = error.as_dict()

    def _monitor_coordinate_correction(self, task: dict) -> None:
        """只在稳定反馈和有界策略允许时，发送一次新的补偿目标。"""
        correction = task["correction"]
        if (not correction["enabled"] or self._shutdown.is_set() or self._cancel_upload.is_set()
                or self._active() is not task or task["status"] != "sent"):
            return
        median_positions, reason = self._coordinate_stable_window(task)
        if median_positions is None:
            correction.update(state="waiting_settle" if reason == "settle_time" else "waiting_stable", reason=reason)
            return
        try:
            feedback = self._evaluate_coordinate(task, median_positions, self._position_time)
        except (KinematicsError, ValueError, TypeError, OverflowError) as error:
            correction["details"] = {"message": str(error)}
            self._unknown_task("稳定坐标反馈无法解算；已停止后续修正", "coordinate_feedback_invalid")
            return
        correction["stable_feedback"] = copy.deepcopy(feedback)
        if feedback["within_tolerance"]:
            correction.update(state="waiting_stable", reason="waiting_acceptance_samples")
            return
        baseline = task.get("_correction_baseline_score")
        if baseline is not None and baseline - feedback["score"] < correction["min_progress"]:
            correction["details"] = {"score_before": baseline, "score_after": feedback["score"],
                                      "min_progress": correction["min_progress"]}
            self._unknown_task("修正后稳定反馈没有足够改善；已停止后续修正", "correction_no_progress")
            return
        if correction["attempts"] >= correction["max_attempts"]:
            self._unknown_task("已达到修正次数上限，执行结果未达标", "correction_attempts_exhausted")
            return
        try:
            targets = plan_correction(
                self._kinematics, task["targets"], task["command_targets"], median_positions,
                self.settings.limits(), task["coordinate_plan"]["start_positions"],
                max_step=correction["max_step"], max_offset=correction["max_offset"],
                max_joint_step=self.settings.coordinate_max_joint_step, deadband=correction["deadband"],
            )
            previous_commands = [task["targets"]] + [entry["command_targets"] for entry in correction["history"]]
            transition = (task["command_targets"], targets)
            if (targets == task["command_targets"]
                    or any(transition == old for old in zip(previous_commands, previous_commands[1:]))):
                raise CorrectionError("correction_cycle", "修正重复已有的有向命令转移，已拒绝")
            self._write_allowed("coordinate")
            self._check_targets(targets)
        except CorrectionError as error:
            correction["details"] = copy.deepcopy(error.details)
            self._unknown_task(error.message, error.code)
            return
        except ArmError as error:
            correction["details"] = error.as_dict()
            self._unknown_task("当前配置或连接不允许继续修正", error.code)
            return
        if (self._shutdown.is_set() or self._cancel_upload.is_set() or task["status"] != "sent"
                or self._active() is not task or self._motion_timed_out(task)):
            return
        correction["attempts"] += 1
        correction.update(state="correcting", reason="bounded_correction")
        correction["history"].append({"attempt": correction["attempts"], "issued_at": now(),
                                      "command_targets": copy.deepcopy(targets),
                                      "score_before": feedback["score"], "feedback_before": copy.deepcopy(feedback)})
        del correction["history"][:-correction["max_attempts"]]
        task["command_targets"] = copy.deepcopy(targets)
        task["_correction_baseline_score"] = feedback["score"]
        task["_coordinate_samples"].clear()
        task.update(consecutive_samples=0, joint_consecutive_samples=0, model_consecutive_samples=0,
                    joint_feedback_confirmed=False, model_feedback_confirmed=False)
        self._exchange(Command.MOVE, move_payload(targets, 20))
        task["_last_command_mono"] = time.monotonic()

    def _command(self, request_id: str, signature: dict, action) -> dict:
        """同一进程内防止重复提交；保留最近 512 个请求，不做自动重发。"""
        fingerprint = json.dumps(signature, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        with self._request_lock:
            previous = self._requests.get(request_id)
            if previous:
                if previous["signature"] != fingerprint:
                    raise ArmError("request_id_conflict", "request_id 已用于不同的命令或参数")
                if previous.get("pending"):
                    raise ArmError("request_in_progress", "同一请求仍在处理，请查询服务状态")
                if "error" in previous:
                    data = previous["error"]
                    raise ArmError(**data)
                result = copy.deepcopy(previous["result"])
                result["deduplicated"] = True
                return result
            self._requests[request_id] = {"signature": fingerprint, "pending": True}
            while len(self._requests) > 512:
                old_key = next((key for key, value in self._requests.items() if not value.get("pending")), None)
                if old_key is None:
                    break
                del self._requests[old_key]
        try:
            result = action()
        except ArmError as error:
            with self._request_lock:
                self._requests[request_id] = {"signature": fingerprint, "error": {
                    "code": error.code, "message": error.message, "status": error.status,
                    "outcome_unknown": error.outcome_unknown, "details": error.details}}
            raise
        except Exception as error:
            with self._lock:
                LOG.exception("命令处理异常")
                self._connected = False
                self.transport.close()
                self._unknown_task("命令处理异常，结果未知")
                data = {"code": "internal_error", "message": "命令处理异常；不要自动重发，请查询状态",
                        "status": 500, "outcome_unknown": True, "details": {}}
                with self._request_lock:
                    self._requests[request_id] = {"signature": fingerprint, "error": data}
            raise ArmError(**data) from error
        with self._request_lock:
            result = {**result, "request_id": request_id, "deduplicated": False}
            self._requests[request_id] = {"signature": fingerprint, "result": copy.deepcopy(result)}
        return result

    def status(self) -> dict:
        with self._lock:
            return {"mode": self.settings.mode, "connection": "connected" if self._connected else "disconnected",
                    "active_task_id": self._active_id, "motion_state": self._motion_state,
                    "upload_in_progress": self._uploading, "last_error": copy.deepcopy(self._last_error),
                    "coordinate_reference": "joint_feedback_model" if self._connected and self._positions is not None else "unavailable",
                    "limits_require_recheck": self._limits_require_recheck,
                    "capabilities": {"hardware_verified": False, "writes_enabled": self.settings.mode == "mock" or self.settings.allow_hardware_writes,
                                     "enabled_features": list(self.settings.enabled_features),
                                     "direct_duration_ms": 20, "absolute_xyz": False,
                                     "coordinate_hardware_supported": True,
                                     "coordinate_backend": "host_ik", "coordinate_preview_supported": True,
                                     "cartesian_feedback": False, "coordinate_model_calibrated": False,
                                     "coordinate_feedback_validation": True, "coordinate_correction_supported": True,
                                     "gripper_close_supported": True, "gripper_force_control": False,
                                     "gripper_contact_feedback": False, "gripper_grasp_detection": False,
                                     "host_deduplication_only": True,
                                     "stop_all_joints_confirmed": False, "group_completion_feedback": False},
                    "servo_limits": [{"id": key, "min": value[0], "max": value[1]} for key, value in self.settings.limits().items()],
                    "feedback_settings": {"poll_interval_s": self.settings.poll_interval_s,
                                          "position_tolerance": self.settings.position_tolerance,
                                          "reached_samples": self.settings.reached_samples},
                    "gripper_settings": self._gripper_settings(),
                    "coordinate_settings": {"max_step_cm": self.settings.coordinate_max_step_cm,
                                            "max_joint_step": self.settings.coordinate_max_joint_step,
                                            "min_joint_delta": self.settings.coordinate_min_delta(),
                                            "max_rounding_error_cm": self.settings.coordinate_max_rounding_error_cm,
                                            "position_tolerance_cm": self.settings.coordinate_position_tolerance_cm,
                                            "pitch_tolerance_deg": self.settings.coordinate_pitch_tolerance_deg,
                                            "correction": self._correction_settings(),
                                            "frame": "base_link", "pitch_policy": "keep_current",
                                            "model": copy.deepcopy(self.settings.kinematics)}}

    def device(self, refresh: bool = False) -> dict:
        with self._lock:
            if refresh:
                self._query_allowed()
                packet = self._exchange(Command.VERSION, reply=True)
                try:
                    device = decode_version(packet.payload)
                except ValueError as error:
                    raise self._invalid_reply(error)
                if device["arm_type"] != "bus":
                    raise self._invalid_reply(ValueError("设备类型发生改变，此服务仅支持总线款"),
                                              "unsupported_arm_type", 503)
                self._device = {**device, "observed_at": now(), "source": self.settings.mode}
            if self._device is None:
                raise ArmError("device_unavailable", "尚无可用设备信息", 503)
            return copy.deepcopy(self._device)

    def positions(self, refresh: bool = True) -> dict:
        with self._lock:
            if refresh:
                self._query_allowed()
                return self._read_positions()
            return self._position_result()

    @staticmethod
    def _kinematics_error(error: KinematicsError) -> ArmError:
        return ArmError(error.code, error.message,
                        403 if error.code == "limits_missing" else 422,
                        details=copy.deepcopy(error.details))

    def pose(self, refresh: bool = True) -> dict:
        """由关节反馈推算模型位置，始终不作为实际末端测量或标定结果。"""
        with self._lock:
            if refresh:
                self._query_allowed()
                feedback = self._read_positions()
            else:
                feedback = self._position_result()
            try:
                pose = self._kinematics.forward(self._positions)
            except KinematicsError as error:
                raise self._kinematics_error(error)
            return {**pose, "estimated": True, "calibrated": False,
                    "source": feedback["source"], "observed_at": feedback["observed_at"],
                    "age_ms": feedback["age_ms"], "fresh": feedback["fresh"],
                    "joint_positions": feedback["positions"]}

    def task(self, task_id: str) -> dict:
        with self._lock:
            if task_id not in self._tasks:
                raise ArmError("task_not_found", "任务不存在或已从进程内历史中移除", 404)
            return self._public_task(self._tasks[task_id])

    def release_task(self, request_id: str, task_id: str, confirm_stationary: bool) -> dict:
        """操作者确认现场静止后只释放软件占用；绝不操作通信通道或发送帧。"""
        if confirm_stationary is not True:
            raise ArmError("invalid_request", "confirm_stationary 必须为 true，且须先在现场确认已静止", 422)
        if type(task_id) is not str or len(task_id) != 32 or any(c not in "0123456789abcdef" for c in task_id):
            raise ArmError("invalid_request", "task_id 必须为 32 位小写十六进制字符串", 422)

        def action():
            with self._lock:
                if task_id not in self._tasks:
                    raise ArmError("task_not_found", "任务不存在或已从进程内历史中移除", 404)
                task = self._tasks[task_id]
                if (self._uploading or self._active_id != task_id or task["status"] != "outcome_unknown"
                        or task["type"] not in {"move", "reset", "coordinate", "gripper_close"}):
                    raise ArmError("task_release_not_allowed", "仅允许释放当前结果未知的直接运动、复位、坐标或夹爪闭合任务", 409)
                self._require_connected()
                task["release"] = {"request_id": request_id, "released_at": now(),
                                   "operator_confirmed_stationary": True}
                task["updated_at"] = task["release"]["released_at"]
                self._active_id = None
                self._motion_state = "unobserved"
                return {"status": "released", "task": self._public_task(task),
                        "command_sent": False, "physically_stopped": None}

        return self._command(request_id, {"command": "release_task", "task_id": task_id,
                                           "confirm_stationary": confirm_stationary}, action)

    def reconnect(self, request_id: str) -> dict:
        def action():
            with self._lock:
                task = self._active()
                if self._uploading or (task is not None and task["status"] != "outcome_unknown"):
                    self._idle()
                old_task_id = self._active_id
                if task:
                    self._unknown_task("正在重建连接，此前动作结果未确认")
                self.transport.close()
                self._groups.clear()
                self._connect()
                self._active_id = None
                return {"status": "connected", "previous_motion_result": "unknown" if old_task_id else "unchanged",
                        "previous_task_id": old_task_id, "physically_stopped": None,
                        "device": copy.deepcopy(self._device), "group_registry_cleared": True}
        return self._command(request_id, {"command": "reconnect"}, action)

    def _gripper_settings(self) -> dict:
        return {"max_target_gap": self.settings.gripper_max_target_gap,
                "goal_tolerance": self.settings.gripper_goal_tolerance,
                "min_closing_delta": self.settings.gripper_min_closing_delta,
                "settle_s": self.settings.gripper_settle_s,
                "stable_s": self.settings.gripper_stable_s,
                "stable_samples": self.settings.gripper_stable_samples,
                "stability_tolerance": self.settings.gripper_stability_tolerance,
                "timeout_s": self.settings.gripper_observation_timeout_s,
                "poll_interval_s": self.settings.poll_interval_s}

    def close_gripper(self, request_id: str, close_target: int, expected_contact_range: dict) -> dict:
        parameters = gripper_parameters(close_target, expected_contact_range)
        target, region = parameters["close_target"], parameters["expected_contact_range"]

        def action():
            with self._lock:
                self._idle()
                self._write_allowed()
                for position in (target, region["min"], region["max"]):
                    self._check_targets({1: position})
                acceptance = {**self._gripper_settings(), "expected_contact_range": copy.deepcopy(region)}
                minimum_time = acceptance["settle_s"] + max(
                    acceptance["stable_s"], acceptance["poll_interval_s"] * (acceptance["stable_samples"] - 1))
                if (region["max"] >= target - acceptance["goal_tolerance"]
                        or target - region["min"] > acceptance["max_target_gap"]
                        or acceptance["timeout_s"] <= minimum_time):
                    raise ArmError("gripper_parameters_invalid",
                                   "预计接触区须低于目标到位区、闭合余量不超过上限，且观察时间须足够", 422,
                                   details={"close_target": target, "acceptance": acceptance})
                if self._shutdown.is_set():
                    raise ArmError("connection_cancelled", "服务正在关闭，夹爪命令已取消", 503)
                self._read_positions()
                start = self._positions[1]
                if target <= start or start + acceptance["min_closing_delta"] > region["min"]:
                    raise ArmError("gripper_invalid_start", "起点须位于预计接触区之前并留足闭合位移；不自动张爪或继续加压", 422,
                                   details={"start_position": start, "close_target": target,
                                            "expected_contact_range": copy.deepcopy(region),
                                            "min_closing_delta": acceptance["min_closing_delta"]})
                if self._shutdown.is_set():
                    raise ArmError("connection_cancelled", "服务正在关闭，夹爪命令已取消", 503)
                task = self._new_task("gripper_close", request_id, {1: target})
                task.update(acceptance=acceptance, start_positions={1: start}, command_targets={1: target},
                            completion_basis="gripper_position_observation", observation_state="observing",
                            completion_reason=None, position_goal_reached=False,
                            grasp_confirmed=None, contact_confirmed=None,
                            message="已提交夹爪闭合目标，等待位置观察；不代表夹持成功")
                task["_gripper_samples"] = []
                task["_gripper_sample_region"] = None
                task["_gripper_min_position"] = task["_gripper_max_position"] = start
                self._exchange(Command.MOVE, move_payload({1: target}, 20))
                task["_last_command_mono"] = time.monotonic()
                return {"status": "sent", "task": self._public_task(task),
                        "completion_basis": "gripper_position_observation", "force_feedback": False,
                        "duration_ms": 20, "duration_guaranteed": False}

        return self._command(request_id, {"command": "gripper_close", **parameters}, action)

    def move(self, request_id: str, targets: dict[int, int], duration_ms: int = 20,
             position_tolerance: int | None = None) -> dict:
        def action():
            with self._lock:
                if position_tolerance is not None and (type(position_tolerance) is not int
                                                       or not 0 <= position_tolerance <= 100):
                    raise ArmError("invalid_request", "position_tolerance 必须为 0～100 的整数", 422)
                self._idle()
                self._write_allowed()
                self._check_targets(targets)
                task = self._new_task("move", request_id, targets)
                if position_tolerance is not None:
                    task["acceptance"]["position_tolerance"] = position_tolerance
                self._exchange(Command.MOVE, move_payload(targets, duration_ms))
                return {"status": "sent", "task": self._public_task(task), "duration_ms": 20,
                        "duration_guaranteed": False}
        return self._command(request_id, {"command": "move", "targets": targets, "duration_ms": duration_ms,
                                          "position_tolerance": position_tolerance}, action)

    def reset(self, request_id: str) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed()
                targets = dict(enumerate(RESET_POSITIONS, 1))
                self._check_targets(targets)
                task = self._new_task("reset", request_id, targets)
                self._exchange(Command.RESET)
                # robot_arm_reset 对六个舵机逐一设置，PWM/总线路径均包含
                # 短延时。稍后再查询，避免在复位处理途中覆盖全局命令状态。
                self._query_not_before = time.monotonic() + 0.15
                return {"status": "sent", "task": self._public_task(task)}
        return self._command(request_id, {"command": "reset"}, action)

    def stop(self, request_id: str) -> dict:
        def action():
            self._cancel_upload.set()
            with self._lock:
                self._write_allowed()
                task = self._active()
                self._exchange(Command.STOP)
                if task:
                    task.update(status="stop_requested", message="已发送停止请求；所有关节物理停止情况未知",
                                updated_at=now(), physically_stopped=None)
                    if task["type"] == "coordinate":
                        task["correction"].update(state="stopped", reason="stop_requested")
                    elif task["type"] == "gripper_close":
                        task["observation_state"] = "stopped"
                self._active_id = None
                self._motion_state = "stopped_unknown"
                return {"status": "stop_requested", "physically_stopped": None,
                        "task": self._public_task(task) if task else None,
                        "limitations": ["出厂固件可能遗漏舵机 6", "停止没有确认回包", "HTTP 成功不证明全部关节停止"]}
        return self._command(request_id, {"command": "stop"}, action)

    def _coordinate_plan(self, delta_ticks: tuple[int, int, int]) -> dict:
        """锁内读取当前位置并规划；拒绝路径不会创建任务或下发运动。"""
        self._idle()
        self._require_connected()
        if self._limits_require_recheck:
            raise ArmError("limits_require_recheck", "偏差修改后需重新核验关节范围、更新配置并重启服务", 403)
        if (not isinstance(delta_ticks, (tuple, list)) or len(delta_ticks) != 3
                or any(type(value) is not int or not -128 <= value <= 127 for value in delta_ticks)
                or not any(delta_ticks)):
            raise ArmError("invalid_request", "坐标增量必须为三个非全零的 0.1 cm 整数步，范围 -128～127", 422)
        self._read_positions()
        try:
            plan = self._kinematics.plan_increment(
                self._positions, [value / 10 for value in delta_ticks], self.settings.limits(),
                max_step_cm=self.settings.coordinate_max_step_cm,
                max_joint_step=self.settings.coordinate_max_joint_step,
                max_rounding_error_cm=self.settings.coordinate_max_rounding_error_cm,
            )
        except KinematicsError as error:
            raise self._kinematics_error(error)
        self._check_targets(plan["targets"])
        plan["position_tolerance"] = self.settings.position_tolerance
        plan["min_joint_delta"] = self.settings.coordinate_min_delta()
        plan["feedback_detectable"] = plan["max_joint_delta"] >= plan["min_joint_delta"]
        plan["start_positions"] = {i: self._positions[i] for i in range(3, 7)}
        effective_tolerance = min(self.settings.coordinate_position_tolerance_cm,
                                  math.hypot(*plan["delta_cm"]) / 2)
        plan["acceptance"] = {
            "configured_position_tolerance_cm": self.settings.coordinate_position_tolerance_cm,
            "effective_position_tolerance_cm": effective_tolerance,
            "pitch_tolerance_deg": self.settings.coordinate_pitch_tolerance_deg,
            "joint_position_tolerance": self.settings.position_tolerance,
            "required_samples": self.settings.reached_samples,
        }
        try:
            predicted = evaluate_feedback(
                self._kinematics, plan["targets"], plan,
                joint_tolerance=self.settings.position_tolerance,
                position_tolerance_cm=effective_tolerance,
                pitch_tolerance_deg=self.settings.coordinate_pitch_tolerance_deg,
            )
        except KinematicsError as error:
            raise self._kinematics_error(error)
        plan["acceptance"]["predicted_within_tolerance"] = predicted["within_tolerance"]
        return plan

    def _correction_settings(self) -> dict:
        return {"enabled": self.settings.coordinate_correction_enabled,
                "max_attempts": self.settings.coordinate_correction_max_attempts,
                "max_step": self.settings.coordinate_correction_max_step,
                "max_offset": self.settings.coordinate_correction_max_offset,
                "deadband": self.settings.coordinate_correction_deadband,
                "settle_s": self.settings.coordinate_correction_settle_s,
                "stable_samples": self.settings.coordinate_correction_stable_samples,
                "stability_tolerance": self.settings.coordinate_correction_stability_tolerance,
                "min_progress": self.settings.coordinate_correction_min_progress}

    def preview_coordinate(self, request_id: str, delta_ticks: tuple[int, int, int]) -> dict:
        def action():
            with self._lock:
                return {"status": "preview", "coordinate_plan": self._coordinate_plan(delta_ticks)}
        return self._command(request_id, {"command": "coordinate_preview", "delta_ticks": delta_ticks}, action)

    def coordinate(self, request_id: str, delta_ticks: tuple[int, int, int]) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("coordinate")
                self._cancel_upload.clear()
                plan = self._coordinate_plan(delta_ticks)
                if not plan["feedback_detectable"]:
                    raise ArmError(
                        "coordinate_below_feedback_resolution",
                        "当前增量未达到配置的坐标最小关节变化门槛；请查看预览或核对 coordinate_min_joint_delta",
                        422, details={"max_joint_delta": plan["max_joint_delta"],
                                      "min_joint_delta": plan["min_joint_delta"],
                                      "position_tolerance": plan["position_tolerance"]},
                    )
                if not plan["acceptance"]["predicted_within_tolerance"]:
                    raise ArmError("coordinate_acceptance_unreachable",
                                   "原始位置量化后的预测结果不满足本次位置或俯仰验收阈值", 422,
                                   details={"acceptance": copy.deepcopy(plan["acceptance"]),
                                            "rounding_error_cm": plan["rounding_error_cm"]})
                try:
                    initial_feedback = evaluate_feedback(
                        self._kinematics, self._positions, plan,
                        joint_tolerance=plan["acceptance"]["joint_position_tolerance"],
                        position_tolerance_cm=plan["acceptance"]["effective_position_tolerance_cm"],
                        pitch_tolerance_deg=plan["acceptance"]["pitch_tolerance_deg"],
                        observed_at=self._position_time,
                    )
                except KinematicsError as error:
                    raise self._kinematics_error(error)
                task = self._new_task("coordinate", request_id, copy.deepcopy(plan["targets"]))
                task.update(coordinate_plan=copy.deepcopy(plan), backend="host_ik",
                            completion_basis="joint_positions_and_model", cartesian_feedback=False,
                            coordinate_feedback=initial_feedback, command_targets=copy.deepcopy(plan["targets"]),
                            joint_feedback_confirmed=False, model_feedback_confirmed=False,
                            joint_consecutive_samples=0, model_consecutive_samples=0,
                            correction={**self._correction_settings(), "attempts": 0, "history": [],
                                        "state": "waiting_settle" if self.settings.coordinate_correction_enabled else "disabled",
                                        "reason": None}, _coordinate_samples=[], _last_command_mono=time.monotonic())
                self._exchange(Command.MOVE, move_payload(plan["targets"], 20))
                task["_last_command_mono"] = time.monotonic()
                return {"status": "sent", "task": self._public_task(task),
                        "coordinate_plan": copy.deepcopy(plan), "delta_cm": list(plan["delta_cm"]),
                        "duration_ms": 20, "duration_guaranteed": False,
                        "completion_feedback": True, "completion_basis": "joint_positions_and_model",
                        "backend": "host_ik", "cartesian_feedback": False}
        return self._command(request_id, {"command": "coordinate", "delta_ticks": delta_ticks}, action)

    def upload_group(self, request_id: str, group_id: int, frames: list[dict]) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("groups")
                for frame in frames:
                    self._check_targets(frame["targets"])
                self._uploading = True
                self._cancel_upload.clear()
                self._groups.pop(group_id, None)
            acknowledged = 0
            try:
                for index, frame in enumerate(frames):
                    with self._lock:
                        if self._cancel_upload.is_set():
                            raise ArmError("upload_cancelled", "动作组上传已取消，部分帧可能已写入", 409, True,
                                           {"group_id": group_id, "frames_acknowledged": acknowledged})
                        payload = group_frame_payload(group_id, len(frames), index, frame["targets"], frame["duration_ms"])
                        self._exchange(Command.GROUP_UPLOAD, payload, reply=True)
                        acknowledged += 1
                with self._lock:
                    self._groups[group_id] = {"group_id": group_id, "frame_count": len(frames),
                                              "uploaded_at": now(), "verification": "acknowledged",
                                              "limits_at_upload": copy.deepcopy(self.settings.limits()),
                                              "duration_sum_ms": sum(frame["duration_ms"] for frame in frames)}
                return {"status": "acknowledged", "group_id": group_id, "frames_acknowledged": acknowledged,
                        "playback_verified": False}
            except ArmError as error:
                error.details.update(group_id=group_id, frames_acknowledged=acknowledged)
                raise
            finally:
                with self._lock:
                    self._uploading = False
        return self._command(request_id, {"command": "upload_group", "group_id": group_id, "frames": frames}, action)

    def groups(self) -> dict:
        with self._lock:
            return {"groups": [{key: copy.deepcopy(value) for key, value in group.items() if key != "limits_at_upload"}
                               for group in self._groups.values()], "source": "service_session",
                    "complete_device_inventory": False}

    def run_group(self, request_id: str, group_id: int, repeat: int) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("groups")
                if self._limits_require_recheck:
                    raise ArmError("limits_require_recheck", "偏差修改后请重新核验关节范围", 403)
                if group_id not in self._groups:
                    raise ArmError("group_not_registered", "先由本服务完整上传动作组；无法查询或验证板内已有动作组")
                if self._groups[group_id]["limits_at_upload"] != self.settings.limits():
                    raise ArmError("group_limits_changed", "关节范围已改变，请按当前范围重新上传动作组")
                task = self._new_task("group", request_id)
                self._exchange(Command.GROUP_RUN, group_run_payload(group_id, repeat))
                task.update(status="running_unconfirmed", group_id=group_id, repeat=repeat,
                            message="动作组运行命令已发送；无完成回包，保持占用直到显式停止")
                return {"status": "sent", "task": self._public_task(task), "completion_feedback": False}
        return self._command(request_id, {"command": "run_group", "group_id": group_id, "repeat": repeat}, action)

    def erase_groups(self, request_id: str) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("groups")
                self._groups.clear()
                self._exchange(Command.GROUP_ERASE, reply=True)
                return {"status": "acknowledged", "scope": "all_device_groups"}
        return self._command(request_id, {"command": "erase_groups"}, action)

    def offsets(self) -> dict:
        with self._lock:
            self._query_allowed()
            packet = self._exchange(Command.OFFSETS_READ, reply=True)
            try:
                values = decode_offsets(packet.payload)
            except ValueError as error:
                raise self._invalid_reply(error)
            return {"offsets": [{"id": key, "offset": value} for key, value in sorted(values.items())],
                    "observed_at": now(), "source": self.settings.mode}

    def set_offsets(self, request_id: str, offsets: dict[int, int]) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("calibration")
                self._groups.clear()
                if self.settings.mode == "serial":
                    self._limits_require_recheck = True
                sent = []
                try:
                    for servo_id, value in sorted(offsets.items()):
                        self._exchange(Command.OFFSET_SET, offset_payload(servo_id, value))
                        sent.append(servo_id)
                except ArmError as error:
                    error.details["servo_ids_sent"] = sent
                    if sent:
                        error.outcome_unknown = True
                    raise
                return {"status": "sent", "servo_ids_sent": sent, "saved": False,
                        "readback_confirmed": False}
        return self._command(request_id, {"command": "set_offsets", "offsets": offsets}, action)

    def save_offsets(self, request_id: str) -> dict:
        def action():
            with self._lock:
                self._idle()
                self._write_allowed("calibration")
                self._exchange(Command.OFFSETS_SAVE, reply=True)
                return {"status": "acknowledged", "scope": "current_device_offsets"}
        return self._command(request_id, {"command": "save_offsets"}, action)
