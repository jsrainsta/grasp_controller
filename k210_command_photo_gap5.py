# Transmission pacing trial: 5 ms per chunk; stable command_photo stays at 80 ms.
# Standalone K210 main.py: keep K1 and add one USB capture command.
# k210_send_photo.py remains the verified K1-only fallback.

import time

try:
    import uos as os
except ImportError:
    import os

try:
    import ujson as json
except ImportError:
    import json

try:
    import ubinascii as binascii
except ImportError:
    import binascii


PHOTO_DIRECTORY = "/sd/photo_transfer"
PHOTO_PATH = PHOTO_DIRECTORY + "/latest.jpg"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
JPEG_QUALITY = 90
CHUNK_SIZE = 384
CHUNK_DELAY_MS = 5
MAX_PHOTO_BYTES = 512 * 1024
MAX_COMMAND_BYTES = 256
READ_CHUNK_BYTES = 64
COMMAND_PREFIX = b"@K210:"
TEST_BUILD = "command_photo_gap5_1"
ID_CHARACTERS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"


def valid_request_id(value):
    return (isinstance(value, str) and 1 <= len(value) <= 32
            and all(char in ID_CHARACTERS for char in value))


def command_error(code, request_id=None):
    return {"type": "command_error", "version": 1,
            "request_id": request_id, "code": code}


def parse_command_line(raw):
    try:
        packet = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeError) as error:
        reply = command_error("invalid_json")
        reply["test_build"] = TEST_BUILD
        reply["message"] = repr(error)
        reply["received_bytes"] = len(raw)
        reply["received_hex"] = binascii.hexlify(raw).decode("ascii")
        return reply
    if not isinstance(packet, dict):
        return command_error("expected_object")
    request_id = packet.get("request_id")
    if not valid_request_id(request_id):
        return command_error("invalid_request_id")
    if type(packet.get("version")) is not int or packet["version"] != 1:
        return command_error("invalid_version", request_id)
    if packet.get("type") not in ("ping", "capture"):
        return command_error("unsupported_command", request_id)
    if set(packet) != {"type", "version", "request_id"}:
        return command_error("unexpected_fields", request_id)
    return packet


class CommandDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False
        self.in_frame = False
        self.prefix_index = 0
        self.ignored_bytes_total = 0

    def feed(self, data):
        commands = []
        for byte in data:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            if not self.in_frame:
                if byte == COMMAND_PREFIX[self.prefix_index]:
                    self.prefix_index += 1
                    if self.prefix_index == len(COMMAND_PREFIX):
                        self.in_frame = True
                        self.prefix_index = 0
                else:
                    self.ignored_bytes_total += self.prefix_index
                    self.prefix_index = 1 if byte == COMMAND_PREFIX[0] else 0
                    if self.prefix_index == 0:
                        self.ignored_bytes_total += 1
                continue
            if byte == 10:
                line = bytes(self.buffer).strip()
                self.buffer = bytearray()
                self.in_frame = False
                commands.append(parse_command_line(line))
            elif len(self.buffer) >= MAX_COMMAND_BYTES:
                self.buffer = bytearray()
                self.discarding = True
                self.in_frame = False
                commands.append(command_error("line_too_long"))
            else:
                self.buffer.append(byte)
        return commands


class CameraCommands:
    """One pending capture, one shared sequence counter, no command queue."""

    def __init__(self, uart, emit_packet=None):
        self.uart = uart
        self.emit = emit_json if emit_packet is None else emit_packet
        self.decoder = CommandDecoder()
        self.seq = 0
        self.busy = False
        self.pending_id = None
        self.recent_ids = []

    def poll(self):
        # Bound work per preview frame / JPEG chunk. Do not wait for a line.
        for _ in range(4):
            pending = self.uart.any()
            if type(pending) is not int or pending < 0:
                raise RuntimeError("UART.any returned an invalid byte count")
            if not pending:
                break
            data = self.uart.read(min(pending, READ_CHUNK_BYTES))
            if not data:
                break
            for packet in self.decoder.feed(data):
                kind = packet["type"]
                request_id = packet.get("request_id")
                if kind == "command_error":
                    self.emit(packet)
                elif kind == "ping":
                    self.emit({
                        "type": "pong", "version": 1, "request_id": request_id,
                        "capabilities": ["capture"],
                        "busy": self.busy or self.pending_id is not None,
                        "uart": repr(self.uart), "test_build": TEST_BUILD,
                        "ignored_bytes_total": self.decoder.ignored_bytes_total,
                    })
                elif request_id in self.recent_ids:
                    self.emit(command_error("duplicate_request", request_id))
                elif self.busy or self.pending_id is not None:
                    self.emit(command_error("busy", request_id))
                else:
                    # Reserve immediately, before handling the rest of a batch.
                    self.pending_id = request_id
                    self.recent_ids.append(request_id)
                    if len(self.recent_ids) > 8:
                        del self.recent_ids[0]

    def begin_capture(self):
        if self.busy:
            raise RuntimeError("A photo is already being captured")
        self.busy = True
        self.seq += 1  # Failed sends consume a sequence number too.
        request_id = self.pending_id
        self.pending_id = None
        if request_id is not None:
            self.emit({"type": "capture_accepted", "version": 1,
                       "request_id": request_id, "seq": self.seq})
        return self.seq, request_id

    def finish_capture(self):
        self.busy = False


def emit_json(packet):
    # This stdout path is the same Type-C path used by the detection script.
    print(json.dumps(packet))


def delay_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def is_directory(path):
    try:
        return (os.stat(path)[0] & 0xF000) == 0x4000
    except OSError:
        return False


def prepare_photo_directory():
    # Do not create a fake /sd folder when the TF card is missing.
    if not is_directory("/sd"):
        raise RuntimeError("TF card not mounted. Power off, insert card, retry.")
    if not is_directory(PHOTO_DIRECTORY):
        os.mkdir(PHOTO_DIRECTORY)


def send_jpeg_file(filename, seq, emit_packet=None, sleep_fn=None,
                   image_width=IMAGE_WIDTH, image_height=IMAGE_HEIGHT,
                   request_id=None, service_fn=None):
    """Send one saved JPEG. Optional callbacks allow PC protocol tests.

    emit_packet receives a dictionary; sleep_fn receives milliseconds.
    Dimensions must describe the saved photo, rather than the LCD preview.
    Errors propagate to the caller, which sends photo_error for this seq.
    """
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 1:
        raise ValueError("seq must be a positive integer")
    if request_id is not None and not valid_request_id(request_id):
        raise ValueError("Invalid request_id")
    for dimension in (image_width, image_height):
        if (not isinstance(dimension, int) or isinstance(dimension, bool)
                or not 1 <= dimension <= 4096):
            raise ValueError("image dimensions must be integers from 1 to 4096")
    if emit_packet is None:
        emit_packet = emit_json
    if sleep_fn is None:
        sleep_fn = delay_ms

    def emit_photo(packet):
        if request_id is not None:
            packet["request_id"] = request_id
        emit_packet(packet)

    size = os.stat(filename)[6]
    if size <= 0 or size > MAX_PHOTO_BYTES:
        raise ValueError("JPEG size must be between 1 and 524288 bytes")
    chunks = (size + CHUNK_SIZE - 1) // CHUNK_SIZE
    adler_a = 1
    adler_b = 0

    with open(filename, "rb") as stream:
        emit_photo({
            "type": "photo_begin",
            "version": 1,
            "seq": seq,
            "image_width": image_width,
            "image_height": image_height,
            "size": size,
            "chunk_size": CHUNK_SIZE,
            "chunks": chunks,
        })
        for index in range(chunks):
            if service_fn is not None:
                service_fn()
            expected = min(CHUNK_SIZE, size - index * CHUNK_SIZE)
            data = stream.read(expected)
            if len(data) != expected:
                raise OSError("JPEG ended before its declared size")
            for byte in data:
                adler_a = (adler_a + byte) % 65521
                adler_b = (adler_b + adler_a) % 65521
            encoded = binascii.b2a_base64(data).strip().decode("ascii")
            emit_photo({
                "type": "photo_chunk",
                "seq": seq,
                "index": index,
                "data": encoded,
            })
            sleep_fn(CHUNK_DELAY_MS)
        if stream.read(1):
            raise OSError("JPEG grew after transmission started")

    checksum = "{:08x}".format((adler_b << 16) | adler_a)
    if service_fn is not None:
        service_fn()
    emit_photo({
        "type": "photo_end",
        "seq": seq,
        "size": size,
        "chunks": chunks,
        "adler32": checksum,
    })
    return size


def show_status(image, text):
    import lcd

    image.draw_string(0, 0, text, color=(0, 255, 0), scale=1)
    lcd.display(image)


def show_result_status(image, text):
    # A display-only failure after photo_end must not retract a completed send.
    try:
        show_status(image, text)
    except Exception as error:
        emit_json({"type": "status", "message": "display_warning",
                   "detail": str(error)})


def capture_vga_photo():
    """Save a clean VGA frame, then restore the small LCD's QVGA preview."""
    import gc
    import sensor

    photo = None
    gc.collect()
    try:
        sensor.set_framesize(sensor.VGA)
        sensor.set_vflip(False)
        sensor.set_hmirror(False)
        sensor.skip_frames(time=600)
        photo = sensor.snapshot()
        width = photo.width()
        height = photo.height()
        if width != IMAGE_WIDTH or height != IMAGE_HEIGHT:
            raise RuntimeError("Expected VGA photo, got {}x{}".format(width, height))
        # Save before adding any display text. The quality argument was
        # confirmed on this board by the independent VGA probe.
        photo.save(PHOTO_PATH, quality=JPEG_QUALITY)
        return width, height
    finally:
        # Reconfiguring the sensor invalidates the old framebuffer image.
        # Release the VGA reference before returning to QVGA; do not resize
        # or copy a second large image just for the LCD.
        photo = None
        gc.collect()
        sensor.set_framesize(sensor.QVGA)
        sensor.set_vflip(False)
        sensor.set_hmirror(False)
        sensor.skip_frames(time=300)


def main():
    import gc
    import sensor
    import lcd
    from hiwonder import hw_key
    from machine import UART

    try:
        uart = UART.repl_uart()
        if uart is None or not hasattr(uart, "any") or not hasattr(uart, "read"):
            raise RuntimeError("The current REPL UART does not expose any/read")
        prepare_photo_directory()
        lcd.init(type=1, freq=15000000, color=lcd.BLACK, invert=0, lcd_type=0)
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_vflip(False)
        sensor.set_hmirror(False)
        sensor.skip_frames(time=1500)
        key = hw_key()
        commands = CameraCommands(uart)
    except Exception as error:
        emit_json({"type": "photo_error", "seq": 0, "message": str(error)})
        raise

    emit_json({
        "type": "status", "message": "photo_ready",
        "image_width": IMAGE_WIDTH, "image_height": IMAGE_HEIGHT,
        "jpeg_quality": JPEG_QUALITY,
        "capabilities": ["capture"], "command_prefix": COMMAND_PREFIX.decode("ascii"),
        "test_build": TEST_BUILD,
    })
    previous_pressed = False
    last_press_ms = None

    while True:
        image = sensor.snapshot()
        commands.poll()
        pressed = key.key_scan(1) == 1
        now = time.ticks_ms()
        debounced = (last_press_ms is None or
                     time.ticks_diff(now, last_press_ms) >= 500)
        key_trigger = pressed and not previous_pressed and debounced
        if commands.pending_id is not None or key_trigger:
            last_press_ms = now
            seq, request_id = commands.begin_capture()
            try:
                show_status(image, "Capturing 640x480...")
                image = None
                gc.collect()
                width, height = capture_vga_photo()
                # Commands received during sensor switching are handled as busy.
                commands.poll()
                # Take a new QVGA frame after the sensor switches back.
                image = sensor.snapshot()
                show_status(image, "Sending photo {}...".format(seq))
                size = send_jpeg_file(PHOTO_PATH, seq,
                                      image_width=width, image_height=height,
                                      request_id=request_id, service_fn=commands.poll)
            except Exception as error:
                packet = {
                    "type": "photo_error", "seq": seq, "message": str(error)
                }
                if request_id is not None:
                    packet["request_id"] = request_id
                emit_json(packet)
                image = sensor.snapshot()
                show_result_status(image, "Photo failed - see receiver")
            else:
                show_result_status(image, "Sent {} bytes".format(size))
            finally:
                commands.finish_capture()
            delay_ms(500)
            image = None
            gc.collect()
        else:
            show_status(image, "K1 or PC: take photo")
        # A held key must be released before another capture can start.
        previous_pressed = pressed
        delay_ms(20)


if __name__ == "__main__":
    main()
