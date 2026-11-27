"""Mock and explicit-open serial channels for the factory LeArm protocol.

Only a silent first position query after a move may be resent once. Motion
commands are never retried. A failed transaction closes the serial channel.
"""

from __future__ import annotations

import logging
import math
import os
import re
import threading
import time
from collections.abc import Callable
from typing import Any

from .protocol import (
    Command, Packet, PacketDecoder, ProtocolError,
    decode_offsets, decode_positions, decode_version,
)


class TransportError(Exception):
    def __init__(self, code: str, message: str, outcome_unknown: bool = False):
        super().__init__(message)
        self.code = code
        self.message = message
        self.outcome_unknown = outcome_unknown


class _DeadlineExceeded(Exception):
    pass


def _positive_seconds(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return value


def _request(packet: bytes) -> Packet:
    if not isinstance(packet, bytes):
        raise ProtocolError("packet must be bytes")
    if len(packet) < 4 or packet[:2] != b"\x55\x55":
        raise ProtocolError("invalid packet header")
    if len(packet) != packet[2] + 2:
        raise ProtocolError("packet length does not match its length field")
    decoded = PacketDecoder().feed(packet)
    if len(decoded) != 1:
        raise ProtocolError("exactly one complete packet is required")
    return decoded[0]


_REPLY_LENGTHS = {
    int(Command.VERSION): 2,
    int(Command.OFFSETS_READ): 12,
    int(Command.POSITIONS_READ): 18,
    int(Command.GROUP_ERASE): 0,
    int(Command.OFFSETS_SAVE): 0,
    int(Command.GROUP_UPLOAD): 0,
}


def _validate_reply(packet: Packet) -> None:
    expected_length = _REPLY_LENGTHS.get(int(packet.command))
    if expected_length is None or len(packet.payload) != expected_length:
        raise ProtocolError("unexpected reply payload length")
    decoders = {
        int(Command.VERSION): decode_version,
        int(Command.OFFSETS_READ): decode_offsets,
        int(Command.POSITIONS_READ): decode_positions,
    }
    decoder = decoders.get(int(packet.command))
    if decoder is not None:
        decoder(packet.payload)


def _bounded_io(operation: Callable[[], Any], deadline: float) -> Any:
    """Bound even flush(), whose OS implementation may ignore write_timeout.

    A daemon worker runs one driver operation only. On expiry the caller detaches
    and closes that serial handle; this worker cannot initiate another write.
    """
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise _DeadlineExceeded()
    done = threading.Event()
    result: list[Any] = []
    errors: list[BaseException] = []

    def invoke() -> None:
        try:
            result.append(operation())
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    threading.Thread(target=invoke, daemon=True).start()
    if not done.wait(max(0.0, deadline - time.monotonic())):
        raise _DeadlineExceeded()
    if errors:
        raise errors[0]
    return result[0]


class SerialTransport:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        write_timeout: float = 1,
        inter_command_delay: float = 0.03,
        serial_factory: Callable[..., Any] | None = None,
    ):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("a physical serial device path is required")
        if re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", port) or "://" in port:
            raise ValueError("URL serial ports are not supported")
        if baudrate != 9600:
            raise ValueError("factory host serial baudrate must be 9600")
        delay = float(inter_command_delay)
        if not math.isfinite(delay) or delay < 0:
            raise ValueError("inter_command_delay must be finite and nonnegative")
        self.port = port
        self.baudrate = baudrate
        self.write_timeout = _positive_seconds(write_timeout, "write_timeout")
        self.inter_command_delay = delay
        self._factory = serial_factory
        self._serial: Any | None = None
        self._lock = threading.Lock()
        self._last_transaction_end = 0.0
        self._last_command: int | None = None

    @property
    def is_open(self) -> bool:
        channel = self._serial
        return channel is not None and bool(channel.is_open)

    @staticmethod
    def _dispose(channel: Any | None) -> None:
        if channel is None:
            return
        # Closing cannot postpone the HTTP deadline indefinitely. The handle is
        # already detached, and no subsequent exchange will use it.
        def close_handle() -> None:
            try:
                channel.close()
            except Exception:
                pass
        worker = threading.Thread(target=close_handle, daemon=True)
        worker.start()
        worker.join(0.05)

    def _close_locked(self) -> None:
        channel, self._serial = self._serial, None
        self._last_command = None
        self._dispose(channel)

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def open(self) -> None:
        with self._lock:
            if self.is_open:
                return
            self._close_locked()
            factory = self._factory
            if factory is None:
                try:
                    import serial
                except ImportError as exc:
                    raise TransportError(
                        "serial_unavailable", "pyserial is not installed"
                    ) from exc
                factory = serial.Serial
            kwargs = {
                "port": None,
                "baudrate": self.baudrate,
                "bytesize": 8,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.05,
                "write_timeout": self.write_timeout,
                "xonxoff": False,
                "rtscts": False,
                "dsrdtr": False,
            }
            if os.name == "posix":
                kwargs["exclusive"] = True
            channel = None
            abandoned = threading.Event()
            try:
                channel = factory(**kwargs)
                # Set before open. Some adapters/drivers can still briefly
                # toggle these lines on opening; this is not a no-reset promise.
                channel.dtr = False
                channel.rts = False
                channel.port = self.port

                def open_driver() -> None:
                    try:
                        channel.open()
                    finally:
                        # A driver may finish opening after the caller's timeout
                        # and after its first close. Close that late handle too.
                        if abandoned.is_set():
                            try:
                                channel.close()
                            except Exception:
                                pass

                _bounded_io(open_driver, time.monotonic() + self.write_timeout)
                if not channel.is_open:
                    raise OSError("serial port did not open")
                self._serial = channel
            except Exception as exc:
                abandoned.set()
                self._dispose(channel)
                raise TransportError(
                    "serial_unavailable", "unable to open the configured serial device"
                ) from exc

    def exchange(
        self,
        packet: bytes,
        expected_command: int | None = None,
        timeout: float = 1,
    ) -> Packet | None:
        request = _request(packet)
        timeout = _positive_seconds(timeout, "timeout")
        if expected_command is not None and int(expected_command) not in _REPLY_LENGTHS:
            raise ProtocolError("this command has no supported reply")
        logger = logging.getLogger(__name__)
        started = time.monotonic()
        deadline = started + timeout
        if not self._lock.acquire(timeout=timeout):
            logger.info("serial_trace cmd=%d result=lock_timeout elapsed_ms=%.1f",
                        request.command, (time.monotonic() - started) * 1000)
            raise TransportError("communication_timeout", "serial channel is busy")
        may_have_sent = False
        stage = "pre_write"
        result = "failed"
        error_type = "-"
        written = 0
        tx_done = None
        received_bytes = 0
        decoded_frames = 0
        matched_frames = 0
        valid_position_reply = False
        receive_retries = 0
        receive_had_error = False
        requery_started = False
        requery_tx_done = None
        requery_reply = None
        # Only the first position query following a move may be repeated.
        # Its reply cannot be the second consecutive in-tolerance sample.
        position_query = (request.command == int(Command.POSITIONS_READ)
                          and expected_command == int(Command.POSITIONS_READ))
        may_requery = position_query and self._last_command == int(Command.MOVE)
        try:
            if not self.is_open:
                raise TransportError("not_connected", "serial channel is not connected")
            channel = self._serial
            assert channel is not None
            send_at = self._last_transaction_end + self.inter_command_delay
            while True:
                now = time.monotonic()
                remaining = deadline - now
                if remaining <= 0:
                    raise _DeadlineExceeded()
                wait_for = send_at - now
                if wait_for <= 0:
                    break
                if wait_for >= remaining:
                    raise _DeadlineExceeded()
                # sleep may wake early, and its clock resolution may differ
                # from monotonic on Windows. Recheck before sending any frame.
                time.sleep(wait_for)
            # Discard stale bytes before this command. After a failed query we
            # close instead of relying on this flush to hide a late response.
            _bounded_io(channel.reset_input_buffer, deadline)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise _DeadlineExceeded()
            write_deadline = min(deadline, time.monotonic() + self.write_timeout)
            channel.write_timeout = min(self.write_timeout, remaining)
            stage = "write"
            may_have_sent = True
            written = _bounded_io(lambda: channel.write(packet), write_deadline)
            if written != len(packet):
                raise OSError("incomplete serial write")
            stage = "flush"
            _bounded_io(channel.flush, write_deadline)
            tx_done = time.monotonic()
            if expected_command is None:
                result = "sent_no_reply_expected"
                return None
            stage = "receive"
            decoder = PacketDecoder()
            # Give bounded reads the full receive budget up front. A worker
            # timeout cannot safely be retried while its read may still run.
            receive_deadline = (deadline + 2 * timeout
                                if position_query else deadline)
            if may_requery:
                # Let the final bounded driver read finish after the protocol
                # deadline so the loop can decide whether a requery is safe.
                # A stuck worker still expires here and never triggers a write.
                receive_deadline += 0.1
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if requery_started:
                        if requery_reply is None:
                            raise _DeadlineExceeded()
                        valid_position_reply = True
                        result = "reply_ok_after_requery"
                        return requery_reply
                    if not position_query or receive_retries >= 2:
                        if not may_requery or received_bytes or receive_had_error:
                            raise _DeadlineExceeded()
                        stage = "requery_quiet"
                        quiet_until = time.monotonic() + min(timeout, 0.2)
                        while time.monotonic() < quiet_until:
                            channel.timeout = max(0.0, min(0.05, quiet_until - time.monotonic()))
                            stale = _bounded_io(lambda: channel.read(32), quiet_until + 0.05)
                            received_bytes += len(stale)
                            if stale:
                                raise ProtocolError("late data before position requery")
                        requery_started = True
                        stage = "requery_write"
                        write_deadline = time.monotonic() + min(self.write_timeout, timeout)
                        channel.write_timeout = min(self.write_timeout, timeout)
                        if _bounded_io(lambda: channel.write(packet), write_deadline) != len(packet):
                            raise OSError("incomplete serial write during position requery")
                        stage = "requery_flush"
                        _bounded_io(channel.flush, write_deadline)
                        requery_tx_done = time.monotonic()
                        deadline = requery_tx_done + timeout
                        receive_deadline = deadline + 0.05
                        stage = "requery_receive"
                        logger.info("serial_trace position_requery=sent tx_done_mono=%.6f",
                                    requery_tx_done)
                        continue
                    receive_retries += 1
                    deadline = (deadline if may_requery else time.monotonic()) + timeout
                    # Recompute below if more than one window has already elapsed.
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        continue
                    logging.getLogger(__name__).warning(
                        "Position feedback delayed; receive retry %d/2", receive_retries)
                channel.timeout = min(0.05, remaining)
                try:
                    chunk = _bounded_io(lambda: channel.read(32), receive_deadline)
                except _DeadlineExceeded:
                    stage = "receive_worker_timeout"
                    raise
                except OSError as error:
                    receive_had_error = True
                    if (requery_started or not position_query
                            or receive_retries >= 2 or not channel.is_open
                            or time.monotonic() >= deadline):
                        raise
                    receive_retries += 1
                    logging.getLogger(__name__).warning(
                        "Position read failed; receive retry %d/2: %r", receive_retries, error)
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
                received_bytes += len(chunk)
                if requery_started and received_bytes > 4 + _REPLY_LENGTHS[int(Command.POSITIONS_READ)]:
                    raise ProtocolError("additional data after position requery")
                for reply in decoder.feed(chunk):
                    decoded_frames += 1
                    if int(reply.command) != int(expected_command):
                        if requery_started:
                            raise ProtocolError("unexpected command after position requery")
                        continue
                    matched_frames += 1
                    stage = "validate_reply"
                    _validate_reply(reply)
                    if requery_started:
                        # Wait out the bounded window and reject duplicate data.
                        # The protocol cannot identify a lone arbitrarily late
                        # reply, so a separate later query is still required.
                        requery_reply = reply
                        stage = "requery_receive"
                        continue
                    valid_position_reply = expected_command == int(Command.POSITIONS_READ)
                    result = "reply_ok"
                    return reply
                if not chunk:
                    time.sleep(min(0.001, max(0.0, deadline - time.monotonic())))
        except TransportError as exc:
            error_type = exc.code
            raise
        except BaseException as exc:
            error_type = type(exc).__name__
            self._close_locked()
            timed_out = isinstance(exc, (_DeadlineExceeded, TimeoutError)) or (
                type(exc).__name__ == "SerialTimeoutException"
            )
            result = "timeout" if timed_out else "error"
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            raise TransportError(
                "communication_timeout" if timed_out else "communication_error",
                "serial transaction timed out" if timed_out else "serial transaction failed",
                outcome_unknown=may_have_sent,
            ) from exc
        finally:
            finished = time.monotonic()
            self._last_transaction_end = finished
            self._last_command = int(request.command) if self.is_open else None
            self._lock.release()
            logger.info(
                "serial_trace cmd=%d expected=%s result=%s stage=%s error=%s "
                "tx_bytes=%d tx_done_mono=%s rx_bytes=%d decoded_frames=%d "
                "matched_frames=%d valid_position_reply=%s receive_retries=%d "
                "requery_eligible=%s requery_tx_done_mono=%s elapsed_ms=%.1f",
                request.command, expected_command, result, stage, error_type,
                written, "-" if tx_done is None else f"{tx_done:.6f}", received_bytes,
                decoded_frames, matched_frames, valid_position_reply, receive_retries,
                may_requery, "-" if requery_tx_done is None else f"{requery_tx_done:.6f}",
                (finished - started) * 1000,
            )


class MockTransport:
    """In-memory protocol peer. It never models real motion safety or timing."""

    RESET_POSITIONS = (226, 500, 177, 129, 408, 500)

    def __init__(self):
        self._open = False
        self._lock = threading.Lock()
        self.positions = dict(enumerate(self.RESET_POSITIONS, start=1))
        self.offsets = dict.fromkeys(range(1, 7), 0)
        self.saved_offsets = dict(self.offsets)
        self.groups: dict[int, dict[int, bytes]] = {}
        self.group_running = False
        self.sent_packets: list[bytes] = []

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        with self._lock:
            self._open = True

    def close(self) -> None:
        with self._lock:
            self._open = False

    @staticmethod
    def _targets(payload: bytes, start: int, count: int) -> dict[int, int]:
        if count < 1 or count > 6:
            raise ProtocolError("invalid servo count")
        targets: dict[int, int] = {}
        for offset in range(start, start + count * 3, 3):
            servo_id = payload[offset]
            value = int.from_bytes(payload[offset + 1:offset + 3], "little")
            if servo_id not in range(1, 7) or servo_id in targets or value > 1000:
                raise ProtocolError("invalid servo target")
            targets[servo_id] = value
        return targets

    def _handle(self, packet: Packet) -> Packet | None:
        try:
            command = Command(packet.command)
        except ValueError as exc:
            raise ProtocolError("unsupported command") from exc
        data = packet.payload
        no_payload = {
            Command.VERSION, Command.OFFSETS_READ, Command.POSITIONS_READ,
            Command.STOP, Command.GROUP_ERASE, Command.OFFSETS_SAVE, Command.RESET,
        }
        if command in no_payload and data:
            raise ProtocolError("unexpected request payload")
        if command == Command.VERSION:
            return Packet(command, bytes((2, 1)))
        if command == Command.OFFSETS_READ:
            return Packet(command, bytes(
                part for servo_id in range(1, 7)
                for part in (servo_id, self.offsets[servo_id] & 0xFF)
            ))
        if command == Command.POSITIONS_READ:
            return Packet(command, b"".join(
                bytes((servo_id,)) + self.positions[servo_id].to_bytes(2, "little")
                for servo_id in range(1, 7)
            ))
        if command == Command.MOVE:
            if len(data) < 3 or len(data) != 3 + data[0] * 3:
                raise ProtocolError("invalid move payload")
            targets = self._targets(data, 3, data[0])
            for servo_id, value in targets.items():
                self.positions[servo_id] = min(value, 700) if servo_id == 1 else value
            return None
        if command == Command.COORDINATE:
            if len(data) != 3:
                raise ProtocolError("invalid coordinate payload")
            # No kinematic or absolute-position claims in this mock.
            return None
        if command == Command.GROUP_RUN:
            if len(data) != 3 or data[0] > 254:
                raise ProtocolError("invalid group-run payload")
            repeat = int.from_bytes(data[1:3], "little")
            if not 1 <= repeat <= 255:
                raise ProtocolError("invalid repeat count for factory firmware")
            self.group_running = True
            # Do not fabricate playback completion or position feedback.
            return None
        if command == Command.STOP:
            self.group_running = False
            return None  # The real firmware has no all-joints-stopped ACK.
        if command == Command.GROUP_ERASE:
            self.groups.clear()
            return Packet(command, b"")
        if command == Command.OFFSET_SET:
            if len(data) != 2 or data[0] not in range(1, 7):
                raise ProtocolError("invalid offset payload")
            value = int.from_bytes(data[1:2], "little", signed=True)
            if not -100 <= value <= 100:
                raise ProtocolError("offset is outside firmware limits")
            self.offsets[data[0]] = value
            return None
        if command == Command.OFFSETS_SAVE:
            self.saved_offsets = dict(self.offsets)
            return Packet(command, b"")
        if command == Command.RESET:
            self.positions = dict(enumerate(self.RESET_POSITIONS, start=1))
            self.group_running = False
            return None
        if command == Command.GROUP_UPLOAD:
            if len(data) != 24:
                raise ProtocolError("group upload must contain a padded 21-byte frame")
            group_id, count, index, servo_count = data[:4]
            if group_id > 254 or count == 0 or index >= count:
                raise ProtocolError("invalid group frame index")
            self._targets(data, 6, servo_count)
            if index == 0:
                self.groups[group_id] = {}
            self.groups.setdefault(group_id, {})[index] = data[3:]
            return Packet(command, b"")
        raise ProtocolError("unsupported command")

    def exchange(
        self,
        packet: bytes,
        expected_command: int | None = None,
        timeout: float = 1,
    ) -> Packet | None:
        _positive_seconds(timeout, "timeout")
        request = _request(packet)
        if expected_command is not None and int(expected_command) not in _REPLY_LENGTHS:
            raise ProtocolError("this command has no supported reply")
        with self._lock:
            if not self._open:
                raise TransportError("not_connected", "mock channel is not connected")
            self.sent_packets.append(packet)
            reply = self._handle(request)
            if expected_command is None:
                return None
            if reply is None or int(reply.command) != int(expected_command):
                self._open = False
                raise TransportError(
                    "communication_timeout", "mock has no matching reply", True
                )
            _validate_reply(reply)
            return reply
