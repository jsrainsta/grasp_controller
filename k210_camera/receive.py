"""Receive real K210 detections. This also runs on Raspberry Pi Python 3."""

import argparse
import json
import math
import time

import serial
from serial.tools import list_ports


DISPLAY_NAMES = {
    "banana_peel": "香蕉皮",
    "battery": "电池",
    "bone": "骨头",
    "brush": "刷子",
    "chopstick": "筷子",
}


def choose_port():
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        raise serial.SerialException("未找到串口，请连接 K210。")
    print("电脑当前的串口：")
    for index, port in enumerate(ports, start=1):
        print("  {}. {} — {}".format(index, port.device, port.description))
    if len(ports) == 1:
        return ports[0].device
    while True:
        choice = input("请输入 K210 的 CH340 对应编号：").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device
        print("请输入列表中的编号。")


def validate_detections(message):
    """Reject malformed packets before passing results to application logic."""
    for field in ("seq", "image_width", "image_height"):
        value = message.get(field)
        minimum = 0 if field == "seq" else 1
        if type(value) is not int or value < minimum:
            raise ValueError("无效的 " + field)
    objects = message.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects 必须是列表")
    target_label = message.get("target_label")
    if target_label is not None and (
            not isinstance(target_label, str) or not target_label):
        raise ValueError("无效的 target_label")
    for obj in objects:
        if not isinstance(obj, dict):
            raise ValueError("目标必须是对象")
        if type(obj.get("class_id")) is not int or obj["class_id"] < 0:
            raise ValueError("无效的 class_id")
        if not isinstance(obj.get("label"), str) or not obj["label"]:
            raise ValueError("无效的 label")
        if target_label is not None and obj["label"] != target_label:
            raise ValueError("目标类别与 target_label 不一致")
        confidence = obj.get("confidence")
        if (type(confidence) not in (int, float)
                or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError("无效的 confidence")
        bbox = obj.get("bbox")
        if (not isinstance(bbox, list) or len(bbox) != 4
                or any(type(value) is not int for value in bbox)
                or bbox[2] <= 0 or bbox[3] <= 0):
            raise ValueError("无效的 bbox")
    return message


def handle_detections(message):
    # This is where subsequent Raspberry Pi application logic can be added.
    objects = message["objects"]
    target_label = message.get("target_label")
    if target_label is None:
        selection = ""
    else:
        selection = "（筛选：{}）".format(
            DISPLAY_NAMES.get(target_label, target_label))
    print("帧 {}{}：{} 个目标".format(message["seq"], selection, len(objects)))
    for obj in objects:
        x, y, w, h = obj["bbox"]
        center_x, center_y = x + w / 2, y + h / 2
        name = DISPLAY_NAMES.get(obj["label"], obj["label"])
        print("  {}，置信度 {:.2f}，目标框 {}，像素中心 ({:.1f}, {:.1f})".format(
            name, obj["confidence"], obj["bbox"], center_x, center_y))


def receive(port, baudrate):
    connection = serial.Serial(baudrate=baudrate, timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    with connection:
        print("已打开 {}，波特率 {}。按 Ctrl+C 停止。".format(port, baudrate))
        buffer = bytearray()
        discarding = False
        last_detection = time.monotonic()
        silence_reported = False
        while True:
            chunk = connection.read_until(b"\n")
            # Warn once per interruption, including when non-detection messages
            # continue arriving. An empty detection packet is still fresh data.
            if time.monotonic() - last_detection > 2 and not silence_reported:
                print("超过 2 秒未收到有效识别消息，之前的结果已失效。")
                silence_reported = True
            if not chunk:
                continue
            if discarding:
                if chunk.endswith(b"\n"):
                    discarding = False
                continue
            buffer.extend(chunk)
            if len(buffer) > 16384:
                print("跳过过长消息，请检查波特率和发送内容。")
                buffer.clear()
                discarding = not chunk.endswith(b"\n")
                continue
            if not buffer.endswith(b"\n"):
                continue
            raw = bytes(buffer).strip()
            buffer.clear()
            if not raw:
                continue
            try:
                message = json.loads(raw.decode("utf-8"))
                if not isinstance(message, dict):
                    raise ValueError("消息必须是对象")
                if message.get("type") == "status":
                    print("K210 状态：", message.get("message"))
                    continue
                if message.get("type") != "detections":
                    continue
                validate_detections(message)
            except (UnicodeDecodeError, ValueError) as error:
                print("跳过无效消息：{}；内容：{!r}".format(error, raw[:120]))
                continue
            last_detection = time.monotonic()
            silence_reported = False
            handle_detections(message)


def main():
    parser = argparse.ArgumentParser(description="接收 K210 的 JSON 识别结果")
    parser.add_argument("--port", help="例如 COM5 或 /dev/ttyUSB0；省略时列出串口")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()
    receive(args.port or choose_port(), args.baudrate)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止接收。")
    except serial.SerialException as error:
        print("串口连接失败：", error)
        print("请关闭 CanMV IDE 和串口助手，确认连接后重试。")
