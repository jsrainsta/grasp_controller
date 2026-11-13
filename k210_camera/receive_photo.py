"""Receive one JPEG at a time from K210 over the verified USB serial path."""

import argparse
import base64
import binascii
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from queue import Empty
import re
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
import zlib

import serial

if __package__:
    from .receive import choose_port
    from .ping_k210 import COMMAND_PREFIX, describe_command_error, exchange_ping
else:
    from receive import choose_port
    from ping_k210 import COMMAND_PREFIX, describe_command_error, exchange_ping


MAX_BYTES = 512 * 1024
CHUNK_SIZE = 384
MAX_LINE = 4096
GAP_TIMEOUT = 5
TRANSFER_TIMEOUT = 300
CAPTURE_HANDSHAKE_TIMEOUT = 12
CAPTURE_BOOT_WAIT = 5.0
CAPTURE_ACK_TIMEOUT = 12
CAPTURE_BEGIN_TIMEOUT = 20


class TransferError(ValueError):
    pass


class CaptureError(RuntimeError):
    pass


class ReceiveCancelled(Exception):
    """Normal GUI shutdown while a command is waiting for UART data."""


class _StoppableConnection:
    def __init__(self, connection, stop_event):
        self.connection, self.stop_event = connection, stop_event

    def _check_stop(self):
        if self.stop_event is not None and self.stop_event.is_set():
            raise ReceiveCancelled()

    def write(self, data):
        self._check_stop()
        return self.connection.write(data)

    def read_until(self, *args, **kwargs):
        self._check_stop()
        data = self.connection.read_until(*args, **kwargs)
        self._check_stop()
        return data


def integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise TransferError("无效的 " + name)
    return value


def jpeg_dimensions(data):
    """Check JPEG markers and read SOF dimensions; not a full image decoder."""
    if len(data) < 4 or data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
        raise TransferError("JPEG 文件头或文件尾不完整")
    position = 2
    sof_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                   0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while position < len(data):
        if data[position] != 0xFF:
            raise TransferError("JPEG 标记格式无效")
        while position < len(data) and data[position] == 0xFF:
            position += 1
        if position >= len(data):
            break
        marker = data[position]
        position += 1
        if marker in (0xD8, 0xD9, 0xDA, 0):
            break
        if marker == 1 or 0xD0 <= marker <= 0xD7:
            continue
        if position + 2 > len(data):
            break
        length = int.from_bytes(data[position:position + 2], "big")
        if length < 2 or position + length > len(data):
            raise TransferError("JPEG 数据段不完整")
        if marker in sof_markers:
            if length < 8:
                raise TransferError("JPEG 尺寸信息不完整")
            height = int.from_bytes(data[position + 3:position + 5], "big")
            width = int.from_bytes(data[position + 5:position + 7], "big")
            if width <= 0 or height <= 0:
                raise TransferError("JPEG 尺寸无效")
            return width, height
        position += length
    raise TransferError("未找到 JPEG 尺寸信息")


class PhotoAssembler:
    def __init__(self):
        self.reset()

    def reset(self):
        self.meta = None
        self.data = bytearray()
        self.next_index = 0
        self.checksum = 1
        self.started_at = 0
        self.last_updated = 0

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        if self.meta is None:
            return None
        reason = None
        if now - self.started_at > TRANSFER_TIMEOUT:
            reason = "整张照片接收超过 5 分钟"
        elif now - self.last_updated > GAP_TIMEOUT:
            reason = "超过 5 秒没有收到当前照片的有效数据"
        if reason:
            self.reset()
        return reason

    def accept(self, message, now=None):
        now = time.monotonic() if now is None else now
        try:
            return self._accept(message, now)
        except TransferError:
            self.reset()
            raise

    def _accept(self, message, now):
        kind = message.get("type")
        if kind == "photo_begin":
            integer(message.get("version"), "version", 1, 1)
            seq = integer(message.get("seq"), "seq", 1, 2 ** 63 - 1)
            width = integer(message.get("image_width"), "image_width", 1, 4096)
            height = integer(message.get("image_height"), "image_height", 1, 4096)
            size = integer(message.get("size"), "size", 4, MAX_BYTES)
            integer(message.get("chunk_size"), "chunk_size", CHUNK_SIZE, CHUNK_SIZE)
            count = (size + CHUNK_SIZE - 1) // CHUNK_SIZE
            integer(message.get("chunks"), "chunks", count, count)
            self.reset()
            self.meta = {"seq": seq, "image_width": width, "image_height": height,
                         "size": size, "chunks": count}
            self.started_at = now
            self.last_updated = now
            return None

        if kind not in ("photo_chunk", "photo_end"):
            return None
        reason = self.expire(now)
        if reason:
            raise TransferError(reason)
        if self.meta is None:
            return None  # We joined mid-transfer; wait for a new photo_begin.
        seq = integer(message.get("seq"), "seq", 1, 2 ** 63 - 1)
        if seq != self.meta["seq"]:
            raise TransferError("照片序号不匹配")

        if kind == "photo_chunk":
            integer(message.get("index"), "index", self.next_index, self.next_index)
            expected = min(CHUNK_SIZE, self.meta["size"] - len(self.data))
            if expected <= 0:
                raise TransferError("收到多余的数据块")
            encoded = message.get("data")
            if not isinstance(encoded, str) or len(encoded) != 4 * ((expected + 2) // 3):
                raise TransferError("Base64 数据长度无效")
            try:
                raw = base64.b64decode(encoded, validate=True)
            except (ValueError, binascii.Error) as error:
                raise TransferError("Base64 数据损坏") from error
            if len(raw) != expected:
                raise TransferError("数据块字节数不匹配")
            self.data.extend(raw)
            self.checksum = zlib.adler32(raw, self.checksum) & 0xFFFFFFFF
            self.next_index += 1
            self.last_updated = now
            return None

        integer(message.get("size"), "size", self.meta["size"], self.meta["size"])
        integer(message.get("chunks"), "chunks", self.meta["chunks"], self.meta["chunks"])
        if len(self.data) != self.meta["size"] or self.next_index != self.meta["chunks"]:
            raise TransferError("照片缺少数据块")
        checksum = message.get("adler32")
        if not isinstance(checksum, str) or not re.fullmatch(r"[0-9a-f]{8}", checksum):
            raise TransferError("校验值格式无效")
        if int(checksum, 16) != self.checksum:
            raise TransferError("照片校验值不匹配")
        dimensions = jpeg_dimensions(self.data)
        if dimensions != (self.meta["image_width"], self.meta["image_height"]):
            raise TransferError("JPEG 实际尺寸与发送信息不匹配")
        result = dict(self.meta)
        result["data"] = bytes(self.data)
        self.reset()
        return result


def save_photo(photo, output):
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    name = "photo_{}_seq{}_{}.jpg".format(stamp, photo["seq"], uuid.uuid4().hex[:8])
    destination = output / name
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=output, prefix="receiving_",
                                         suffix=".part", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(photo["data"])
        temporary.replace(destination)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink()
    return destination


def show_photo(path):
    try:
        if sys.platform == "win32":
            os.startfile(str(path))
            return
        program = "open" if sys.platform == "darwin" else "xdg-open"
        if not shutil.which(program):
            print("未找到图片查看器，请在文件管理器中打开保存的照片。")
            return
        subprocess.Popen([program, str(path)], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    except OSError as error:
        print("无法启动图片查看器，请手动打开照片：", error)


class CaptureSession:
    """Accept only the acknowledged photo belonging to one fresh request."""

    def __init__(self, request_id, sent_at=None):
        self.request_id = request_id
        self.ack_deadline = (time.monotonic() if sent_at is None else sent_at) + CAPTURE_ACK_TIMEOUT
        self.begin_deadline = None
        self.seq = None
        self.begun = False
        self.assembler = PhotoAssembler()

    def fail(self, reason):
        self.assembler.reset()
        if self.seq is not None:
            reason += "；拍照请求已接受，但照片未完整接收，板端结果未知。请勿自动重发。"
        raise CaptureError(reason)

    def check_deadlines(self, now=None):
        now = time.monotonic() if now is None else now
        if self.seq is None and now >= self.ack_deadline:
            self.fail("等待本次 capture_accepted 超时；本次请求是否被板端执行尚未确认")
        if self.seq is not None and not self.begun and now >= self.begin_deadline:
            self.fail("等待本次 photo_begin 超时")
        reason = self.assembler.expire(now)
        if reason:
            self.fail(reason)

    def accept(self, message, now=None):
        now = time.monotonic() if now is None else now
        self.check_deadlines(now)
        kind = message.get("type")
        if kind == "status" and message.get("message") == "photo_ready":
            self.fail("拍照请求发出后 K210 重新启动，本次请求已中断")
        if kind == "command_error":
            if message.get("request_id") in (None, self.request_id):
                self.fail(describe_command_error(message))
            return None
        if message.get("request_id") != self.request_id:
            return None  # Other requests and K1 photos cannot satisfy this request.
        if kind not in ("capture_accepted", "photo_begin", "photo_chunk", "photo_end", "photo_error"):
            return None  # Interleaved PONG/status packets do not disturb a photo.
        try:
            seq = integer(message.get("seq"), "seq", 1, 2 ** 63 - 1)
            if kind == "capture_accepted":
                integer(message.get("version"), "version", 1, 1)
                if self.seq is not None:
                    if seq != self.seq:
                        self.fail("同一拍照请求收到不一致的确认序号")
                    return None  # Never extend the begin deadline for duplicate ACKs.
                self.seq = seq
                self.begin_deadline = now + CAPTURE_BEGIN_TIMEOUT
                return None
            if self.seq is None:
                self.fail("尚未收到本次 capture_accepted，就收到对应照片数据或错误")
            if seq != self.seq:
                self.fail("本次照片序号与 capture_accepted 不匹配")
            if kind == "photo_error":
                self.fail("K210 拍照或发送失败：" + str(message.get("message", "未提供原因")))
            if kind == "photo_begin":
                if self.begun:
                    self.fail("同一拍照请求收到重复的 photo_begin")
                self.assembler.accept(message, now)
                self.begun = True
                return None
            if not self.begun:
                self.fail("尚未收到本次 photo_begin，就收到数据块或结束包")
            photo = self.assembler.accept(message, now)
            if photo is not None:
                photo["request_id"] = self.request_id
            return photo
        except TransferError as error:
            self.fail("本次照片校验失败：" + str(error))


def _check_capture_pong(pong):
    capabilities = pong.get("capabilities")
    if not isinstance(capabilities, list) or "capture" not in capabilities:
        raise CaptureError("K210 当前程序不支持 capture；请换成指令拍照程序，"
                           "旧 echo 测试或 K1 拍照程序不能执行本轮请求。")
    if pong.get("busy") is True:
        raise CaptureError("K210 当前忙于其他拍照请求，本次没有发送 capture；请等待后再手动运行。")
    if type(pong.get("busy")) is not bool:
        raise CaptureError("K210 的 PONG 缺少有效 busy 状态，本次没有发送 capture。")


def _receive_requested_photo(connection, output, show=False, on_photo=None,
                             boot_wait=CAPTURE_BOOT_WAIT, on_state=None,
                             on_requested_photo=None):
    # Finish PING before sending capture, so PING cannot consume photo packets.
    # A persistent session passes zero: no reopening or boot delay per photo.
    if boot_wait:
        print("保持串口连接，先等待 {:g} 秒，再发送 PING 和拍照指令。".format(boot_wait), flush=True)
    time.sleep(boot_wait)
    try:
        pong = exchange_ping(connection, uuid.uuid4().hex[:16], CAPTURE_HANDSHAKE_TIMEOUT)
    except (RuntimeError, TimeoutError) as error:
        raise CaptureError("拍照指令握手失败：{}\n本次尚未发送 capture；请保留诊断输出，"
                           "先单独运行 ping_k210.py 检查通信。".format(error)) from error
    _check_capture_pong(pong)
    request_id = uuid.uuid4().hex[:16]
    payload = COMMAND_PREFIX + (json.dumps(
        {"type": "capture", "version": 1, "request_id": request_id},
        separators=(",", ":"), ensure_ascii=True) + "\n").encode("ascii")
    request_started = time.perf_counter()
    if connection.write(payload) != len(payload):
        raise CaptureError("capture 没有完整写入串口，本次请求是否执行未知；请勿自动重发。")
    session = CaptureSession(request_id)
    print("已发送一次拍照指令，请求编号：", request_id, flush=True)
    print("等待 K210 确认并发送对应照片；无需按 K1。", flush=True)
    if on_state is not None:
        on_state("capturing", "拍照指令已发送，请保持相机和药品稳定。")
    buffer = bytearray()
    discarding = False
    photo_begin_at = None
    while True:
        session.check_deadlines()
        chunk = connection.read_until(b"\n", size=MAX_LINE + 1)
        session.check_deadlines()
        if not chunk:
            continue
        if discarding:
            if chunk.endswith(b"\n"):
                discarding = False
            continue
        buffer.extend(chunk)
        if len(buffer) > MAX_LINE:
            buffer.clear()
            discarding = not chunk.endswith(b"\n")
            if session.begun:
                session.fail("照片传输中收到过长消息")
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
                raise ValueError("消息必须是 JSON 对象")
        except (UnicodeDecodeError, ValueError):
            if session.begun:
                session.fail("照片传输中收到损坏消息")
            continue
        previous_seq = session.seq
        photo = session.accept(message)
        if previous_seq is None and session.seq is not None:
            print("K210 已接受本次拍照请求，照片序号：", session.seq, flush=True)
        if (message.get("type") == "photo_begin"
                and message.get("request_id") == request_id and session.begun):
            photo_begin_at = time.perf_counter()
            meta = session.assembler.meta
            print("开始接收照片 {}：{}×{}，{} 字节，{} 块。".format(
                meta["seq"], meta["image_width"], meta["image_height"],
                meta["size"], meta["chunks"]), flush=True)
            if on_state is not None:
                on_state("receiving", "照片已拍好，正在传输；可以移开相机。")
        elif (message.get("type") == "photo_chunk"
              and message.get("request_id") == request_id
              and session.assembler.meta is not None
              and session.assembler.next_index % 10 == 0):
            print("已接收 {}/{} 块。".format(
                session.assembler.next_index, session.assembler.meta["chunks"]), flush=True)
        if photo is not None:
            transfer_finished = time.perf_counter()
            path = save_photo(photo, output)
            saved_at = time.perf_counter()
            print("本次请求照片接收成功：{} 字节，校验通过。".format(photo["size"]))
            print("已保存：", path)
            print("本次分段耗时（不含启动等待、PING 握手和 OCR）：", flush=True)
            print("  拍照及回传准备：{:.2f} 秒".format(photo_begin_at - request_started), flush=True)
            print("  传图与校验：{:.2f} 秒".format(transfer_finished - photo_begin_at), flush=True)
            print("  保存照片：{:.2f} 秒".format(saved_at - transfer_finished), flush=True)
            print("  发出拍照指令到保存完成：{:.2f} 秒".format(saved_at - request_started), flush=True)
            if on_requested_photo is not None:
                on_requested_photo(path, {key: photo[key] for key in (
                    "request_id", "seq", "size", "image_width", "image_height")})
            if on_photo is not None:
                try:
                    on_photo(path)
                except Exception as error:
                    print("照片已保存，但后续处理未能启动：", error)
            if show:
                show_photo(path)
            return path


def receive_photos(port, baudrate, output, show=False, once=False, on_photo=None,
                   request_capture=False, boot_wait=CAPTURE_BOOT_WAIT, stop_event=None,
                   capture_requests=None, on_state=None, on_requested_photo=None):
    """Receive photos; stop_event allows a GUI to stop the continuous K1 loop."""
    if (isinstance(boot_wait, bool) or not isinstance(boot_wait, (int, float))
            or not math.isfinite(boot_wait) or not 0 <= boot_wait <= 30):
        raise ValueError("boot_wait 必须在 0 到 30 秒之间。")
    if stop_event is not None:
        if request_capture:
            raise ValueError("stop_event 只用于连续 K1 接收模式。")
        if stop_event.is_set():
            return None
    if capture_requests is not None and (request_capture or once):
        raise ValueError("capture_requests 只用于持续接收模式。")
    if request_capture or capture_requests is not None:
        connection = serial.Serial(baudrate=baudrate, timeout=0.25, write_timeout=2)
    else:
        connection = serial.Serial(baudrate=baudrate, timeout=0.25)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    assembler = PhotoAssembler()
    buffer = bytearray()
    discarding = False
    old_program_reported = False
    last_input_at = None
    with connection:
        print("已打开 {}，波特率 {}。".format(port, baudrate))
        if request_capture:
            print("保存目录：", Path(output).expanduser().resolve())
            return _receive_requested_photo(connection, output, show, on_photo, boot_wait,
                                             on_requested_photo=on_requested_photo)
        command_connection = _StoppableConnection(connection, stop_event)
        if capture_requests is not None:
            if on_state is not None:
                on_state("starting", "正在准备相机，请等待；启动等待只执行一次。")
            print("保持串口连接，启动时先等待 {:g} 秒，再确认相机状态。".format(boot_wait), flush=True)
            if stop_event is None:
                time.sleep(boot_wait)
            elif stop_event.wait(boot_wait):
                return None
            try:
                pong = exchange_ping(command_connection, uuid.uuid4().hex[:16], CAPTURE_HANDSHAKE_TIMEOUT)
                _check_capture_pong(pong)
            except ReceiveCancelled:
                return None
            if isinstance(pong.get("test_build"), str):
                print("K210 程序版本：", pong["test_build"], flush=True)
            print("相机指令已就绪，等待软件拍照请求。", flush=True)
            if on_state is not None:
                on_state("ready", "相机已就绪，可以点击“拍照并识别”，也可按 K1。")
        else:
            print("准备好后按 K210 的 K1 拍照发送；按 Ctrl+C 停止接收。")
        print("保存目录：", Path(output).expanduser().resolve())
        while stop_event is None or not stop_event.is_set():
            if (capture_requests is not None and assembler.meta is None
                    and not buffer and not discarding):
                try:
                    capture_requests.get_nowait()
                except Empty:
                    pass
                else:
                    if on_state is not None:
                        on_state("capturing", "正在确认相机状态，请保持相机和药品稳定。")
                    result_message = "照片已保存，可以继续拍照。"
                    try:
                        _receive_requested_photo(command_connection, output, show, on_photo,
                                                 boot_wait=0, on_state=on_state,
                                                 on_requested_photo=on_requested_photo)
                    except ReceiveCancelled:
                        return None
                    except CaptureError as error:
                        print("本次软件拍照未完成：", error, flush=True)
                        result_message = "本次拍照未完成：{}；请等预览恢复后再手动拍照。".format(error)
                    finally:
                        capture_requests.task_done()
                    if on_state is not None:
                        on_state("ready", result_message)
                    continue
            chunk = connection.read_until(b"\n", size=MAX_LINE + 1)
            if stop_event is not None and stop_event.is_set():
                return None  # Closing the window discards an unfinished photo.
            if chunk:
                last_input_at = time.monotonic()
            elif (capture_requests is not None and assembler.meta is None
                  and (buffer or discarding) and last_input_at is not None
                  and time.monotonic() - last_input_at > GAP_TIMEOUT):
                buffer.clear()
                discarding = False
                print("忽略未结束的串口输出，继续处理拍照请求。", flush=True)
            reason = assembler.expire()
            if reason:
                buffer.clear()
                print("丢弃不完整照片：{}。请重新按 K1。".format(reason))
                if on_state is not None:
                    on_state("ready", "照片接收超时，未保存；请手动重拍。")
            if not chunk:
                continue
            if discarding:
                if chunk.endswith(b"\n"):
                    discarding = False
                continue
            buffer.extend(chunk)
            if len(buffer) > MAX_LINE:
                buffer.clear()
                discarding = not chunk.endswith(b"\n")
                assembler.reset()
                print("跳过过长消息，请重新按 K1。")
                if on_state is not None:
                    on_state("ready", "接收消息过长，未保存；请手动重拍。")
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
                    raise ValueError("消息必须是 JSON 对象")
            except (UnicodeDecodeError, ValueError):
                if assembler.meta is not None:
                    assembler.reset()
                    print("照片传输中收到损坏消息，已丢弃。请重新按 K1。")
                    if on_state is not None:
                        on_state("ready", "照片数据损坏，未保存；请手动重拍。")
                else:
                    print("忽略非照片输出：{!r}".format(raw[:100]))
                continue
            kind = message.get("type")
            if (capture_requests is not None
                    and kind in {"photo_begin", "photo_chunk", "photo_end", "photo_error"}
                    and message.get("request_id") is not None):
                continue  # Late/foreign command photos cannot become K1 photos.
            if kind == "status":
                if message.get("message") == "photo_ready":
                    assembler.reset()
                    print("K210 已就绪，请按 K1 发送照片。")
                    if on_state is not None:
                        on_state("ready", "相机预览已恢复，可以拍照。")
                continue
            if kind == "photo_error":
                assembler.reset()
                print("K210 拍照或发送失败：", message.get("message"))
                if on_state is not None:
                    on_state("ready", "本次拍照失败，请查看终端信息后手动重试。")
                continue
            if kind == "detections":
                if not old_program_reported:
                    print("当前收到旧识别程序的数据，请先在 K210 保存照片发送脚本。")
                    old_program_reported = True
                continue
            try:
                photo = assembler.accept(message)
            except TransferError as error:
                print("照片校验失败：{}。请重新按 K1。".format(error))
                if on_state is not None:
                    on_state("ready", "照片校验失败，未保存；请手动重拍。")
                continue
            if kind == "photo_begin" and assembler.meta is not None:
                meta = assembler.meta
                print("开始接收照片 {}：{}×{}，{} 字节，{} 块。".format(
                    meta["seq"], meta["image_width"], meta["image_height"],
                    meta["size"], meta["chunks"]))
                if on_state is not None:
                    on_state("receiving", "照片已拍好，正在传输；可以移开相机。")
            elif kind == "photo_chunk" and assembler.meta is not None:
                if assembler.next_index % 10 == 0:
                    print("已接收 {}/{} 块。".format(
                        assembler.next_index, assembler.meta["chunks"]))
            if photo is not None:
                path = save_photo(photo, output)
                print("照片接收成功：{} 字节，校验通过。".format(photo["size"]))
                print("已保存：", path)
                if on_photo is not None:
                    try:
                        on_photo(path)
                    except Exception as error:
                        print("照片已保存，但后续处理未能启动：", error)
                if show:
                    show_photo(path)
                if once:
                    return path
                print("可再次按 K1 发送下一张照片。")
                if on_state is not None:
                    on_state("ready", "照片已保存，可以继续拍照。")


def main():
    parser = argparse.ArgumentParser(description="接收 K210 单张 JPEG 照片")
    parser.add_argument("--port", help="例如 COM5；省略时列出串口供选择")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "images")
    parser.add_argument("--show", action="store_true", help="保存后请求桌面图片查看器打开")
    parser.add_argument("--once", action="store_true", help="成功保存一张照片后退出")
    parser.add_argument("--capture", action="store_true", help="主动发送一次拍照指令，接收对应照片后退出，无需按 K1")
    parser.add_argument("--verify-jpeg", action="store_true",
                        help="单次测速时，保存后独立检查 JPEG 是否能完整解码")
    parser.add_argument("--boot-wait", type=float, default=CAPTURE_BOOT_WAIT,
                        help="指令拍照时，打开串口后先等多少秒再握手，默认 5，范围 0..30")
    parser.add_argument("--ocr", action="store_true", help="保存后在后台自动识别药名")
    parser.add_argument("--ocr-output", type=Path,
                        default=Path(__file__).resolve().parent / "ocr_results",
                        help="自动 OCR 的 TXT / JSON 保存目录")
    parser.add_argument("--catalog", type=Path,
                        default=Path(__file__).resolve().parent / "medicines.json",
                        help="自动 OCR 使用的演示药品名单")
    parser.add_argument("--min-name-score", type=float, default=0.8,
                        help="自动 OCR 完整名称匹配的最低模型分数，默认 0.8")
    parser.add_argument("--target", help="自动 OCR 要筛选的药品演示编号，例如 med_002；需要 --ocr")
    args = parser.parse_args()
    if args.verify_jpeg and (not args.capture or args.ocr or args.show):
        parser.error("--verify-jpeg 需要 --capture，且不能与 --ocr 或 --show 一起使用。")
    if args.target is not None and not args.ocr:
        parser.error("--target 需要与 --ocr 一起使用。")
    if not math.isfinite(args.boot_wait) or not 0 <= args.boot_wait <= 30:
        parser.error("--boot-wait 必须在 0 到 30 秒之间。")
    port = args.port or choose_port()
    worker = None
    try:
        if args.ocr:
            from auto_ocr import OCRWorker

            worker = OCRWorker(args.ocr_output, catalog_path=args.catalog,
                               min_name_score=args.min_name_score, target_id=args.target)
            print("已启用自动 OCR：接收并保存照片后自动识别，结果保存到：",
                  worker.output_dir)
        path = receive_photos(port, args.baudrate, args.output,
                              show=args.show, once=args.once,
                              on_photo=worker.submit if worker is not None else None,
                              request_capture=args.capture, boot_wait=args.boot_wait)
        if args.verify_jpeg:
            if path is None:
                raise SystemExit("未保存照片，无法验证 JPEG。")
            if __package__:
                from .jpeg_integrity import check_jpeg
            else:
                from jpeg_integrity import check_jpeg
            integrity = check_jpeg(path)
            if not integrity["valid"]:
                raise SystemExit("JPEG 解码校验失败：" + str(integrity))
            print("JPEG 解码校验通过：640×480。")
    finally:
        if worker is not None:
            worker.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止接收，已保存的照片保留；未完成传输的照片不会保存。")
    except (serial.SerialException, OSError) as error:
        print("接收失败：", error)
        print("请确认连接、保存目录，并关闭占用该串口的 IDE 或接收程序。")
        raise SystemExit(1)
    except CaptureError as error:
        print("本次指令拍照未完成：", error)
        print("未保存或提交不完整照片；请根据提示检查后手动重试。")
        raise SystemExit(1)
    except ModuleNotFoundError as error:
        print("缺少自动 OCR 依赖：", error.name)
        print("请按 OCR实验说明.md 安装 requirements-ocr.txt 后重试。")
        raise SystemExit(1)
    except ValueError as error:
        print("启动参数或药品名单无效：", error)
        raise SystemExit(1)
