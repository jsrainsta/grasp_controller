"""Send one harmless PING and verify the matching PONG, without OCR or images."""

import argparse
import json
import math
import time
import uuid

import serial

if __package__:
    from .receive import choose_port
else:
    from receive import choose_port


MAX_LINE = 2048
COMMAND_PREFIX = b"@K210:"
DIAGNOSTIC_BYTES = 512


def describe_ping_receive(received_bytes, sample, tail):
    """Expose ignored boot/noise bytes on failure without changing parsing."""
    lines = ["等待期间串口收到：{} 字节。".format(received_bytes)]
    if received_bytes:
        lines.append("接收样本：前 {} 字节（最多保留 {} 字节）。".format(
            len(sample), DIAGNOSTIC_BYTES))
        lines.append("接收 HEX：" + bytes(sample).hex())
        lines.append("接收内容：" + repr(bytes(sample).decode("utf-8", errors="replace")))
        if received_bytes > len(sample):
            lines.append("接收末尾样本：最后 {} 字节。".format(len(tail)))
            lines.append("末尾 HEX：" + bytes(tail).hex())
            lines.append("末尾内容：" + repr(bytes(tail).decode("utf-8", errors="replace")))
            omitted = max(0, received_bytes - len(sample) - len(tail))
            if omitted:
                lines.append("头尾样本之间省略 {} 字节。".format(omitted))
    return "\n".join(lines)


def describe_command_error(packet):
    lines = ["K210 返回错误：{}；{}".format(
        packet.get("code"), packet.get("message", ""))]
    received_hex = packet.get("received_hex")
    if isinstance(received_hex, str):
        lines.append("板端收到（不含换行）：{} 字节".format(packet.get("received_bytes")))
        lines.append("板端原始 HEX：" + received_hex)
        try:
            raw = bytes.fromhex(received_hex)
        except ValueError:
            pass
        else:
            lines.append("板端收到的内容：" + repr(raw.decode("utf-8", errors="replace")))
    elif packet.get("code") == "invalid_json":
        lines.append("此回复没有原始数据；请确认已将更新后的 echo 程序保存到板上 main.py。")
    return "\n".join(lines)


def exchange_ping(connection, request_id, timeout=8.0):
    if (not isinstance(request_id, str) or not 1 <= len(request_id) <= 32
            or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
                   for char in request_id)):
        raise ValueError("无效的 request_id。")
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or not 0 < timeout <= 60):
        raise ValueError("timeout 必须大于 0 且不超过 60 秒。")
    payload = COMMAND_PREFIX + (json.dumps(
        {"type": "ping", "version": 1, "request_id": request_id},
        separators=(",", ":"), ensure_ascii=True) + "\n").encode("ascii")
    if connection.write(payload) != len(payload):
        raise OSError("PING 没有完整写入串口。")
    print("已发送 PING，测试编号：", request_id, flush=True)
    print("发送内容：", payload.decode("ascii").rstrip("\n"), flush=True)
    print("发送字节数：{}（含末尾换行）".format(len(payload)), flush=True)
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    discarding = False
    unidentified_error = None
    received_bytes = 0
    sample = bytearray()
    tail = bytearray()
    while time.monotonic() < deadline:
        chunk = connection.read_until(b"\n", size=MAX_LINE + 1)
        if not chunk:
            continue
        received_bytes += len(chunk)
        if len(sample) < DIAGNOSTIC_BYTES:
            sample.extend(chunk[:DIAGNOSTIC_BYTES - len(sample)])
        tail.extend(chunk)
        if len(tail) > DIAGNOSTIC_BYTES:
            del tail[:-DIAGNOSTIC_BYTES]
        if discarding:
            if chunk.endswith(b"\n"):
                discarding = False
            continue
        buffer.extend(chunk)
        if len(buffer) > MAX_LINE:
            buffer.clear()
            discarding = not chunk.endswith(b"\n")
            continue
        if not buffer.endswith(b"\n"):
            continue
        raw = bytes(buffer).strip()
        buffer.clear()
        try:
            packet = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            continue  # Boot messages are not command replies.
        if not isinstance(packet, dict):
            continue
        if packet.get("type") == "command_error":
            if packet.get("request_id") == request_id:
                raise RuntimeError(describe_command_error(packet))
            if packet.get("request_id") is None:
                # A boot-time partial line may predate this PING. Keep waiting
                # for its exact ID; report the raw failed line if none arrives.
                unidentified_error = describe_command_error(packet)
        if (packet.get("type") == "pong"
                and type(packet.get("version")) is int and packet["version"] == 1
                and packet.get("request_id") == request_id):
            return packet
    diagnostic = describe_ping_receive(received_bytes, sample, tail)
    if unidentified_error is not None:
        raise RuntimeError("等待本次 PONG 超时。\n" + unidentified_error + "\n" + diagnostic)
    raise TimeoutError("等待 PONG 超时，没有收到与本次测试编号对应的回复。\n" + diagnostic)


def ping_k210(port, baudrate=115200, timeout=8.0, boot_wait=0.5):
    if (isinstance(boot_wait, bool) or not isinstance(boot_wait, (int, float))
            or not math.isfinite(boot_wait) or not 0 <= boot_wait <= 30):
        raise ValueError("boot_wait 必须在 0 到 30 秒之间。")
    connection = serial.Serial(baudrate=baudrate, timeout=0.2, write_timeout=2)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    with connection:
        print("已打开 {}，波特率 {}。".format(port, baudrate))
        # Keep this same port open throughout the startup wait and PING.
        # Do not flush buffered boot messages: they are useful on failure.
        print("保持串口连接，先等待 {:g} 秒，再发送一次 PING。".format(boot_wait), flush=True)
        time.sleep(boot_wait)
        packet = exchange_ping(connection, uuid.uuid4().hex[:16], timeout)
        print("已收到对应 PONG，测试编号：", packet["request_id"])
        if isinstance(packet.get("uart"), str):
            print("K210 当前串口：", packet["uart"])
        if isinstance(packet.get("test_build"), str):
            print("板端测试版本：", packet["test_build"])
        ignored = packet.get("ignored_bytes_total")
        if type(ignored) is int and ignored >= 0:
            print("板端累计跳过帧外杂字节：", ignored)
        print("USB 双向指令通信验证成功。")
        return packet


def main():
    parser = argparse.ArgumentParser(description="验证电脑与 K210 的 PING / PONG 往返")
    parser.add_argument("--port", help="例如 COM5；省略时列出串口供选择")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="独立上电测试先沿用已验证的 115200")
    parser.add_argument("--timeout", type=float, default=8.0, help="等待回复的秒数，默认 8")
    parser.add_argument("--boot-wait", type=float, default=0.5,
                        help="打开串口后先等多少秒再发 PING，默认 0.5，最大 30")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error("--timeout 必须大于 0 且不超过 60 秒。")
    if not 0 < args.baudrate <= 5000000:
        parser.error("--baudrate 无效。")
    if not math.isfinite(args.boot_wait) or not 0 <= args.boot_wait <= 30:
        parser.error("--boot-wait 必须在 0 到 30 秒之间。")
    ping_k210(args.port or choose_port(), args.baudrate, args.timeout, args.boot_wait)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止 PING 测试。")
    except (serial.SerialException, OSError, RuntimeError, ValueError) as error:
        print("PING 测试未通过：", error)
        print("请保留完整输出；当前指令拍照程序和 command_echo 都支持 PING。")
        print("测试时关闭 IDE 和其他串口接收程序；超时原因需结合接收字节和板端状态判断。")
        raise SystemExit(1)
