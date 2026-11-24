"""Wire protocol for the LeArm STM32 factory firmware.

Positions are raw bus-servo units, not angles.  The protocol has no checksum or
request identifier; response matching belongs to the transport layer.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct


HEADER = b"\x55\x55"
MAX_PACKET_SIZE = 32
MAX_LENGTH = MAX_PACKET_SIZE - len(HEADER)
MAX_PAYLOAD_SIZE = MAX_LENGTH - 2
RESET_POSITIONS = (226, 500, 177, 129, 408, 500)


class ProtocolError(ValueError):
    """A frame or an argument cannot be represented by the factory protocol."""


class Command(IntEnum):
    VERSION = 1
    OFFSETS_READ = 2
    MOVE = 3
    COORDINATE = 4
    GROUP_RUN = 6
    STOP = 7
    GROUP_ERASE = 8
    OFFSET_SET = 10
    OFFSETS_SAVE = 11
    RESET = 12
    POSITIONS_READ = 13
    GROUP_UPLOAD = 25


def _integer(value: object, minimum: int, maximum: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ProtocolError(f"{name} must be in {minimum}..{maximum}")
    return value


def _bytes(value: object, name: str) -> bytes:
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise ProtocolError(f"{name} must be bytes")
    return bytes(value)


@dataclass(frozen=True)
class Packet:
    command: int
    payload: bytes

    def __post_init__(self) -> None:
        _integer(self.command, 0, 255, "command")
        if not isinstance(self.payload, bytes):
            raise ProtocolError("payload must be immutable bytes")
        if len(self.payload) > MAX_PAYLOAD_SIZE:
            raise ProtocolError("payload exceeds the 32-byte packet limit")


def encode_packet(command: int, payload: bytes = b"") -> bytes:
    """Encode ``55 55 length command payload``; length excludes the header."""
    _integer(command, 0, 255, "command")
    payload = _bytes(payload, "payload")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolError("payload exceeds the 32-byte packet limit")
    return HEADER + bytes((len(payload) + 2, command)) + payload


class PacketDecoder:
    """Incremental decoder with a bounded buffer, including for large reads.

    A plausible but incomplete frame stays pending until further bytes arrive.
    Invalid lengths and bytes before the next header are discarded.  As the
    firmware protocol has no checksum, byte noise inside a plausible frame
    cannot reliably be distinguished from its payload.
    """

    def __init__(self, max_buffer_size: int = 4096) -> None:
        self.max_buffer_size = _integer(
            max_buffer_size, MAX_PACKET_SIZE, 1_048_576, "max_buffer_size"
        )
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[Packet]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise ProtocolError("data must be bytes")
        view = memoryview(data).cast("B")
        packets: list[Packet] = []
        offset = 0
        while offset < len(view):
            take = min(self.max_buffer_size - len(self._buffer), len(view) - offset)
            self._buffer.extend(view[offset : offset + take])
            offset += take
            self._extract(packets)
        return packets

    def _extract(self, packets: list[Packet]) -> None:
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start < 0:
                # Preserve a possible first header byte across reads.
                if self._buffer[-1] == HEADER[0]:
                    self._buffer[:] = HEADER[:1]
                else:
                    self._buffer.clear()
                return
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 3:
                return
            length = self._buffer[2]
            if not 2 <= length <= MAX_LENGTH:
                del self._buffer[:1]
                continue
            packet_size = length + len(HEADER)
            if len(self._buffer) < packet_size:
                return
            packets.append(Packet(self._buffer[3], bytes(self._buffer[4:packet_size])))
            del self._buffer[:packet_size]


def _targets(targets: dict[int, int]) -> list[tuple[int, int]]:
    if not isinstance(targets, dict) or not 1 <= len(targets) <= 6:
        raise ProtocolError("targets must contain 1..6 distinct servo IDs")
    checked = []
    for servo_id, position in targets.items():
        checked.append(
            (
                _integer(servo_id, 1, 6, "servo_id"),
                _integer(position, 0, 1000, "position"),
            )
        )
    return sorted(checked)


def move_payload(targets: dict[int, int], duration_ms: int = 20) -> bytes:
    """Build a move payload; bus firmware ignores duration and uses 20 ms.

    Mechanical limits, including the factory clamp for servo 1, must also be
    enforced by the service before sending commands.
    """
    items = _targets(targets)
    _integer(duration_ms, 1, 65535, "duration_ms")
    return struct.pack("<BH", len(items), duration_ms) + b"".join(
        struct.pack("<BH", servo_id, position) for servo_id, position in items
    )


def coordinate_payload(dx: int, dy: int, dz: int) -> bytes:
    """Encode signed increments in units of 0.1 cm; not absolute coordinates."""
    values = tuple(_integer(value, -128, 127, name) for value, name in (
        (dx, "dx"), (dy, "dy"), (dz, "dz")
    ))
    return struct.pack("bbb", *values)


def group_run_payload(group_id: int, repeat: int) -> bytes:
    _integer(group_id, 0, 254, "group_id")
    # The wire field has 16 bits; action_group_run stores only 8 bits.
    _integer(repeat, 1, 255, "repeat")
    return struct.pack("<BH", group_id, repeat)


def group_frame_payload(
    group_id: int,
    total_frames: int,
    index: int,
    targets: dict[int, int],
    duration_ms: int,
) -> bytes:
    _integer(group_id, 0, 254, "group_id")
    _integer(total_frames, 1, 255, "total_frames")
    _integer(index, 0, total_frames - 1, "index")
    frame = move_payload(targets, duration_ms)
    # action_group_save copies exactly 21 bytes regardless of the servo count.
    # Sending the padding explicitly prevents stale packet data being saved.
    frame = frame.ljust(21, b"\x00")
    return bytes((group_id, total_frames, index)) + frame


def offset_payload(servo_id: int, value: int) -> bytes:
    _integer(servo_id, 1, 6, "servo_id")
    _integer(value, -100, 100, "offset")
    return struct.pack("Bb", servo_id, value)


def decode_version(payload: bytes) -> dict[str, str | int]:
    payload = _bytes(payload, "version payload")
    if len(payload) != 2 or payload[0] not in (1, 2):
        raise ProtocolError("version response must contain arm type 1/2 and version")
    return {
        "arm_type": "bus" if payload[0] == 2 else "pwm",
        "firmware_version": payload[1],
    }


def decode_positions(payload: bytes) -> dict[int, int]:
    payload = _bytes(payload, "positions payload")
    if len(payload) != 18:
        raise ProtocolError("positions response must contain all 6 servos")
    positions: dict[int, int] = {}
    for offset in range(0, len(payload), 3):
        servo_id, position = struct.unpack_from("<BH", payload, offset)
        _integer(servo_id, 1, 6, "servo_id")
        _integer(position, 0, 1000, "position")
        if servo_id in positions:
            raise ProtocolError("positions response contains a duplicate servo ID")
        positions[servo_id] = position
    return positions


def decode_offsets(payload: bytes) -> dict[int, int]:
    payload = _bytes(payload, "offsets payload")
    if len(payload) != 12:
        raise ProtocolError("offsets response must contain all 6 servos")
    offsets: dict[int, int] = {}
    for offset in range(0, len(payload), 2):
        servo_id, value = struct.unpack_from("Bb", payload, offset)
        _integer(servo_id, 1, 6, "servo_id")
        _integer(value, -100, 100, "offset")
        if servo_id in offsets:
            raise ProtocolError("offsets response contains a duplicate servo ID")
        offsets[servo_id] = value
    return offsets
