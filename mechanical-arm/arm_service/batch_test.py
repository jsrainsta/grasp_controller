"""人工启动的 2/6 号批量实物测试；仅通过现有 HTTP 服务控制。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import uuid

from .client import ArmClient, ArmClientError
from .config import PROTOCOL_LIMITS, Settings

SERVOS = (2, 6)
STEP = 20
HOLD_S = 1.0
OTHER_DRIFT = 5


class BatchTestError(RuntimeError):
    """本轮中止；异常后不能推断机械臂已经停止。"""


def _positions_dict(positions):
    if type(positions) is not dict or set(positions) != set(range(1, 7)):
        raise BatchTestError("位置必须完整包含六个不同舵机")
    for servo_id, position in positions.items():
        if (type(servo_id) is not int or type(position) is not int
                or not PROTOCOL_LIMITS[servo_id][0] <= position <= PROTOCOL_LIMITS[servo_id][1]):
            raise BatchTestError("位置反馈超出协议范围或格式错误")
    return dict(positions)


def serial_positions(reply):
    try:
        data = reply["data"]
        rows = data["positions"]
        if (reply["mode"] != "serial" or data["source"] != "serial"
                or data["fresh"] is not True or type(rows) is not list or len(rows) != 6):
            raise BatchTestError("需要来自串口的新六路位置反馈")
        result = {item["id"]: item["position"] for item in rows}
        return _positions_dict(result)
    except (KeyError, TypeError) as error:
        raise BatchTestError("位置响应格式错误") from error


def _serial_idle(reply):
    try:
        data = reply["data"]
        if (reply["mode"] != "serial" or data["mode"] != "serial"
                or data["connection"] != "connected"
                or data["active_task_id"] is not None
                or data["upload_in_progress"] or data["limits_require_recheck"]
                or data["last_error"] is not None
                or data["capabilities"]["enabled_features"] != []):
            raise BatchTestError("服务须为已连接、空闲且扩展功能关闭的串口服务")
        return data
    except (KeyError, TypeError) as error:
        raise BatchTestError("服务状态响应格式错误") from error


def build_plan(settings: Settings, positions: dict[int, int], amplitude: int = 60) -> dict:
    if settings.mode != "serial":
        raise BatchTestError("实物计划只适用于 serial 配置")
    if type(amplitude) is not int or amplitude not in (40, 60):
        raise BatchTestError("本阶段幅度只能为 40 或 60 个原始单位")
    positions = _positions_dict(positions)
    config = asdict(settings)
    config.update(allow_hardware_writes=True, enabled_features=[],
                  position_tolerance=10, reached_samples=2,
                  servo_limits={str(i): [p - amplitude, p + amplitude] if i in SERVOS else [p, p]
                                for i, p in positions.items()})
    plan = {"schema": 1, "plan_id": uuid.uuid4().hex,
            "centers": {str(i): p for i, p in positions.items()},
            "amplitude": amplitude, "step": STEP, "hold_s": HOLD_S, "config": config}
    validate_plan(plan)
    return plan


def validate_plan(plan) -> Settings:
    keys = {"schema", "plan_id", "centers", "amplitude", "step", "hold_s", "config"}
    if type(plan) is not dict or set(plan) != keys:
        raise BatchTestError("计划字段不完整或包含未知字段")
    if (type(plan["schema"]) is not int or plan["schema"] != 1
            or type(plan["plan_id"]) is not str or not re.fullmatch(r"[a-f0-9]{32}", plan["plan_id"])
            or type(plan["amplitude"]) is not int or plan["amplitude"] not in (40, 60)
            or type(plan["step"]) is not int or plan["step"] != STEP
            or type(plan["hold_s"]) not in (int, float) or plan["hold_s"] != HOLD_S):
        raise BatchTestError("计划版本、幅度、步长或等待时间不符合本阶段要求")
    centers = plan["centers"]
    if type(centers) is not dict or set(centers) != {str(i) for i in range(1, 7)}:
        raise BatchTestError("计划必须记录六个起点")
    _positions_dict({int(i): p for i, p in centers.items()})
    try:
        settings = Settings(**plan["config"])
    except (TypeError, ValueError) as error:
        raise BatchTestError(f"计划配置无效：{error}") from error
    expected = {int(i): (p - plan["amplitude"], p + plan["amplitude"]) if int(i) in SERVOS else (p, p)
                for i, p in centers.items()}
    if (settings.mode != "serial" or not settings.allow_hardware_writes or settings.enabled_features
            or settings.servo_limits != expected
            or settings.position_tolerance != 10 or settings.reached_samples != 2):
        raise BatchTestError("计划配置与已记录的 2/6 号窄范围不一致")
    return settings


def segment_targets(start: int, end: int, step: int = STEP) -> list[int]:
    if type(start) is not int or type(end) is not int or type(step) is not int or step <= 0:
        raise BatchTestError("分段参数必须为整数，步长必须为正数")
    count = math.ceil(abs(end - start) / step)
    return [start + round((end - start) * i / count) for i in range(1, count + 1)]


def sweep_targets(center: int, amplitude: int = 60, step: int = STEP) -> list[int]:
    result = []
    previous = center
    for endpoint in (center + amplitude, center, center - amplitude, center):
        result.extend(segment_targets(previous, endpoint, step))
        previous = endpoint
    return result


class BatchRunner:
    def __init__(self, client, plan, emit=lambda event: None, sleep=time.sleep, monotonic=time.monotonic):
        self.client = client
        self.plan = plan
        self.settings = validate_plan(plan)
        self.emit = emit
        self.sleep = sleep
        self.monotonic = monotonic
        self.commands_sent = 0
        self.attempts = 0
        self.anchors = None
        self.step_index = 0

    def _status(self):
        data = _serial_idle(self.client.get("status"))
        try:
            rows = data["servo_limits"]
            actual = {item["id"]: (item["min"], item["max"]) for item in rows}
            feedback = data["feedback_settings"]
            if (data["capabilities"]["writes_enabled"] is not True
                    or len(rows) != 6 or actual != self.settings.servo_limits
                    or feedback["position_tolerance"] != self.settings.position_tolerance
                    or feedback["reached_samples"] != self.settings.reached_samples
                    or feedback["poll_interval_s"] != self.settings.poll_interval_s):
                raise BatchTestError("运行服务的目标范围或反馈设置与计划不一致")
        except (KeyError, TypeError) as error:
            raise BatchTestError("服务配置响应格式错误") from error

    def _read(self):
        return serial_positions(self.client.get("positions?refresh=true"))

    def _check(self, positions, active_id):
        for servo_id, value in positions.items():
            if servo_id != active_id and abs(value - self.anchors[servo_id]) > OTHER_DRIFT:
                raise BatchTestError(f"非目标舵机 {servo_id} 较其观测基线变化超过 {OTHER_DRIFT}，停止后续动作")

    def _wait_task(self, task_id, servo_id, target):
        deadline = self.monotonic() + self.settings.motion_timeout_s + 3
        original_timeout = self.client.timeout
        try:
            while True:
                remaining = deadline - self.monotonic()
                if remaining <= 0:
                    raise BatchTestError("到位查询超时，停止后续动作；未保证机械臂停止")
                self.client.timeout = min(original_timeout, remaining)
                reply = self.client.get(f"tasks/{task_id}")
                task = reply.get("data", {})
                if (reply.get("mode") != "serial" or task.get("task_id") != task_id
                        or task.get("type") != "move" or task.get("targets") != {str(servo_id): target}):
                    raise BatchTestError("任务反馈与本次运动不一致")
                if task.get("status") == "reached" and task.get("feedback_confirmed") is True:
                    return task
                if task.get("status") != "sent":
                    raise BatchTestError(f"任务未确认到位：{task.get('status')}；停止后续动作")
                self.sleep(min(0.2, remaining))
        finally:
            self.client.timeout = original_timeout

    def _move(self, servo_id, target, phase):
        self._status()
        before = self._read()
        self._check(before, servo_id)
        low, high = self.settings.servo_limits[servo_id]
        if not low <= target <= high:
            raise BatchTestError("目标超出本轮已确认范围")
        self.step_index += 1
        request_id = f"batch-{self.plan['plan_id']}-{self.step_index}"
        detail = {"step": self.step_index, "servo_id": servo_id, "target": target,
                  "phase": phase, "request_id": request_id, "before": before,
                  "movement_expected": abs(target - before[servo_id]) > self.settings.position_tolerance}
        self.emit({"event": "post_attempted", **detail})
        self.attempts += 1
        reply = self.client.post("move", {"targets": [{"id": servo_id, "position": target}],
                                          "duration_ms": 20}, request_id=request_id)
        data = reply.get("data", {})
        task = data.get("task", {})
        task_id = task.get("task_id")
        if (reply.get("mode") != "serial" or data.get("status") != "sent"
                or data.get("deduplicated") is not False
                or type(task_id) is not str or not re.fullmatch(r"[a-f0-9]{32}", task_id)
                or task.get("type") != "move" or task.get("targets") != {str(servo_id): target}):
            raise BatchTestError("提交响应异常，命令执行结果可能未知")
        self.commands_sent += 1
        self.emit({"event": "sent", **detail, "task_id": task_id})
        reached = self._wait_task(task_id, servo_id, target)
        self.emit({"event": "reached", "step": self.step_index, "task": reached})
        self.sleep(self.plan["hold_s"])
        after = self._read()
        self._check(after, servo_id)
        if abs(after[servo_id] - target) > self.settings.position_tolerance:
            raise BatchTestError("等待后的位置偏差超过到位容差，停止后续动作")
        self.anchors[servo_id] = after[servo_id]
        self.emit({"event": "settled_reading", **detail, "after": after,
                   "feedback_change": after[servo_id] - before[servo_id],
                   "movement_feedback_observed": abs(after[servo_id] - before[servo_id]) > self.settings.position_tolerance})
        return after

    def run(self):
        try:
            self._status()
            initial = self._read()
            for servo_id, value in initial.items():
                if servo_id in SERVOS:
                    low, high = self.settings.servo_limits[servo_id]
                    if not low <= value <= high:
                        raise BatchTestError(f"舵机 {servo_id} 初始位置已超出本轮范围，停止")
                elif abs(value - self.plan["centers"][str(servo_id)]) > OTHER_DRIFT:
                    raise BatchTestError("打开串口后非测试关节起点变化过大，停止")
            self.anchors = dict(initial)
            self.emit({"event": "started", "initial": initial, "plan_id": self.plan["plan_id"]})
            endpoints = []
            for servo_id in SERVOS:
                positions = self._read()
                self._check(positions, servo_id)
                center = self.plan["centers"][str(servo_id)]
                for target in segment_targets(positions[servo_id], center):
                    self._move(servo_id, target, "align_center")
                for target in sweep_targets(center, self.plan["amplitude"]):
                    after = self._move(servo_id, target, "sweep")
                    if target in (center - self.plan["amplitude"], center + self.plan["amplitude"], center):
                        endpoints.append({"servo_id": servo_id, "target": target, "positions": after})
            result = {"commands_sent": self.commands_sent, "endpoints": endpoints}
            self.emit({"event": "completed", **result})
            return result
        except BaseException as error:
            try:
                self.emit({"event": "aborted", "reason": str(error) or type(error).__name__,
                           "post_attempts": self.attempts, "commands_sent": self.commands_sent,
                           "physically_stopped": None})
            except Exception:
                pass
            raise


def _load_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise BatchTestError("JSON 字段重复")
            result[key] = value
        return result
    return json.loads(Path(path).read_text(encoding="utf-8-sig"), object_pairs_hook=pairs)


def show_plan(plan):
    print("本轮只控制 2 号腕部与 6 号底座，一次一个关节。", flush=True)
    for servo_id in SERVOS:
        center = plan["centers"][str(servo_id)]
        low, high = plan["config"]["servo_limits"][str(servo_id)]
        print(f"{servo_id}号：记录中心 {center}，目标范围 {low}～{high}；正向→中心→反向→中心。", flush=True)
    print("目标步长最多20，每段反馈到位后等待1秒再读取；不保证平滑速度或物理停止。", flush=True)
    print("重开后先在同一范围内分段对齐记录中心；异常不自动返程、重发、重连或发送停止。", flush=True)


def _client(settings):
    return ArmClient(f"http://127.0.0.1:{settings.port}/v1/arm")


def main(argv=None):
    parser = argparse.ArgumentParser(description="LeArm 2/6号人工批量测试（仅HTTP，不直接打开串口）")
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="仅查询，生成独立配置与计划")
    prepare.add_argument("--config", required=True, help="现有串口基础配置，例如只读配置")
    prepare.add_argument("--output", default="config.serial.batch-test.json")
    prepare.add_argument("--amplitude", type=int, choices=(40, 60), default=60)
    execute = commands.add_parser("run", help="显示完整范围，经一次人工确认后执行")
    execute.add_argument("--plan", default="config.serial.batch-test.plan.json")
    args = parser.parse_args(argv)
    try:
        if args.command == "prepare":
            settings = Settings.load(args.config)
            if settings.mode != "serial":
                raise BatchTestError("准备阶段需要串口配置")
            output = Path(args.output)
            plan_path = output.with_suffix(".plan.json")
            if output.exists() or plan_path.exists():
                raise BatchTestError("配置或计划已存在；不会覆盖，请先核对原计划")
            client = _client(settings)
            _serial_idle(client.get("status"))
            plan = build_plan(settings, serial_positions(client.get("positions?refresh=true")), args.amplitude)
            with output.open("x", encoding="utf-8") as file:
                json.dump(plan["config"], file, ensure_ascii=False, indent=2)
            with plan_path.open("x", encoding="utf-8") as file:
                json.dump(plan, file, ensure_ascii=False, indent=2)
            show_plan(plan)
            print(f"已生成：{output} 和 {plan_path}；准备阶段未发送运动命令。", flush=True)
            return 0

        plan_path = Path(args.plan)
        plan = _load_json(plan_path)
        settings = validate_plan(plan)
        report_path = plan_path.with_suffix(".run.jsonl")
        if report_path.exists():
            raise BatchTestError("此计划已有运行记录，拒绝重复执行；请保留原记录")
        runner = BatchRunner(_client(settings), plan)
        runner._status()
        show_plan(plan)
        print("确认所列完整范围内无本体碰撞或线缆牵扯，PC模式和电源开关就绪后输入 RUN；取消按Ctrl+C。", flush=True)
        with open("/dev/tty", "r") as console:
            if console.readline().strip() != "RUN":
                print("已取消，未发送运动命令。", flush=True)
                return 0
        # 独占创建既保留日志，又阻止同一计划同时/再次运行。
        with report_path.open("x", encoding="utf-8") as file:
            def emit(event):
                record = {"observed_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"), **event}
                file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                file.flush()
                os.fsync(file.fileno())
                if event["event"] == "settled_reading":
                    note = "" if event["movement_expected"] else "（起始已在容差内，不计为新增运动验证）"
                    print(f"第{event['step']}步：{event['servo_id']}号 目标{event['target']}，反馈{event['after'][event['servo_id']]} {note}", flush=True)
                elif event["event"] == "aborted":
                    print("本轮已中止后续命令；机械臂停止情况未知。", flush=True)
            runner.emit = emit
            print(f"本轮记录：{report_path}", flush=True)
            result = runner.run()
            print(f"批量结束，共提交{result['commands_sent']}条单关节命令。", flush=True)
            print("最终端点记录：", json.dumps(result["endpoints"], ensure_ascii=False), flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n已取消后续命令；若已开始运动，Ctrl+C不保证机械臂停止，必要时关闭舵机电源。", file=sys.stderr)
        return 130
    except (BatchTestError, ArmClientError, OSError, ValueError, TypeError, KeyError) as error:
        print(f"测试未完成：{error}。请保留记录，不要自动重试；未保证机械臂停止。", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
