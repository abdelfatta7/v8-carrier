"""Pi side of the serial protocol in docs/protocol.md.

Mirrors firmware/lib/protocol/ byte for byte; docs/protocol_vectors.json
(written by the C++ test) is replayed through this module by
test/test_protocol_py.py. Pure Python, no ROS imports.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

SYNC0 = 0xAA
SYNC1 = 0x55
MAX_PAYLOAD = 64

# Message ids, Pi -> ESP32
SET_WHEEL_VEL = 0x01
SET_SERVO = 0x02
REQUEST_STATUS = 0x03
HEARTBEAT = 0x04
CLEAR_ESTOP = 0x05
SET_VACUUM_MODE = 0x06
SET_BRUSH = 0x07
SET_STREAM = 0x08
# ESP32 -> Pi
STATUS = 0x80
ACK = 0x81
LOG = 0x82

ACK_OK = 0
ACK_REJECTED_ESTOP = 1
ACK_BAD_ARG = 2
ACK_BUSY = 3

ESTOP_LATCHED = 1 << 0
ESTOP_BUMPER = 1 << 1
ESTOP_CLIFF = 1 << 2
ESTOP_LOW_BATTERY = 1 << 3
ESTOP_HEARTBEAT = 1 << 4
ESTOP_CAUSE_NAMES = {
    ESTOP_BUMPER: "bumper",
    ESTOP_CLIFF: "cliff",
    ESTOP_LOW_BATTERY: "low_battery",
    ESTOP_HEARTBEAT: "heartbeat",
}

VAC_OFF, VAC_SOFT, VAC_HARD, VAC_UNKNOWN = 0, 1, 2, 3
VACUUM_MODE_NAMES = {VAC_OFF: "off", VAC_SOFT: "soft", VAC_HARD: "hard", VAC_UNKNOWN: "unknown"}
VACUUM_MODE_IDS = {v: k for k, v in VACUUM_MODE_NAMES.items()}
VAC_IDLE, VAC_BUSY, VAC_FAULT = 0, 1, 2
VACUUM_STATUS_NAMES = {VAC_IDLE: "idle", VAC_BUSY: "busy", VAC_FAULT: "fault"}
VACUUM_STATUS_MIC_RUNNING = 1 << 7

YAW_NOT_AVAILABLE = 0x7FFF


# ---- CRC ----------------------------------------------------------------------
def _make_crc_table() -> List[int]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor-out."""
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# ---- Framing ------------------------------------------------------------------
def encode_frame(msg_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    head = bytes((msg_id & 0xFF, len(payload)))
    crc = crc16_ccitt(head + payload)
    return bytes((SYNC0, SYNC1)) + head + payload + struct.pack("<H", crc)


class FrameParser:
    """Byte-stream parser; feed() yields (id, payload) for each valid frame."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        self._buf.extend(data)
        frames: List[Tuple[int, bytes]] = []
        while True:
            start = self._buf.find(bytes((SYNC0, SYNC1)))
            if start < 0:
                # Keep a trailing 0xAA in case the 0x55 is still to come.
                if self._buf and self._buf[-1] == SYNC0:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                return frames
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 4:
                return frames
            msg_id, length = self._buf[2], self._buf[3]
            if length > MAX_PAYLOAD:
                del self._buf[:2]
                continue
            total = 4 + length + 2
            if len(self._buf) < total:
                return frames
            payload = bytes(self._buf[4:4 + length])
            received = struct.unpack_from("<H", self._buf, 4 + length)[0]
            if crc16_ccitt(bytes(self._buf[2:4 + length])) == received:
                frames.append((msg_id, payload))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                del self._buf[:2]  # resync after the bad sync pair


# ---- Commands (Pi -> ESP32) -----------------------------------------------------
def set_wheel_vel(left_mm_s: int, right_mm_s: int) -> bytes:
    return encode_frame(SET_WHEEL_VEL, struct.pack("<hh", _clamp_i16(left_mm_s), _clamp_i16(right_mm_s)))


def set_servo(angle_deg: int) -> bytes:
    return encode_frame(SET_SERVO, struct.pack("<B", max(0, min(180, int(angle_deg)))))


def request_status() -> bytes:
    return encode_frame(REQUEST_STATUS)


def heartbeat(seq: int) -> bytes:
    return encode_frame(HEARTBEAT, struct.pack("<B", seq & 0xFF))


def clear_estop() -> bytes:
    return encode_frame(CLEAR_ESTOP)


def set_vacuum_mode(mode: int) -> bytes:
    if mode not in (VAC_OFF, VAC_SOFT, VAC_HARD):
        raise ValueError("mode must be 0, 1 or 2")
    return encode_frame(SET_VACUUM_MODE, struct.pack("<B", mode))


def set_brush(duty_pct: int) -> bytes:
    return encode_frame(SET_BRUSH, struct.pack("<B", max(0, min(100, int(duty_pct)))))


def set_stream(enable: bool) -> bytes:
    return encode_frame(SET_STREAM, struct.pack("<B", 1 if enable else 0))


def _clamp_i16(v: int) -> int:
    return max(-32768, min(32767, int(round(v))))


# ---- Status / ack / log (ESP32 -> Pi) -------------------------------------------
_STATUS_FMT = "<IiihhhhhhhB4HHhHBBBBBB"
STATUS_SIZE = struct.calcsize(_STATUS_FMT)
assert STATUS_SIZE == 47


@dataclass
class Status:
    t_ms: int = 0
    left_ticks: int = 0
    right_ticks: int = 0
    gyro_mrad_s: Tuple[int, int, int] = (0, 0, 0)
    accel_mm_s2: Tuple[int, int, int] = (0, 0, 0)
    yaw_1e4_rad: int = YAW_NOT_AVAILABLE
    bumper_bits: int = 0
    cliff_mv: Tuple[int, int, int, int] = (0, 0, 0, 0)
    pack_mv: int = 0
    pack_ma: int = 0
    mic_level: int = 0
    estop_flags: int = 0
    estop_latch_cause: int = 0
    servo_angle_deg: int = 0
    vacuum_mode: int = VAC_UNKNOWN
    vacuum_status: int = VAC_IDLE
    brush_duty_pct: int = 0

    # Derived helpers -----------------------------------------------------------
    @property
    def estop_latched(self) -> bool:
        return bool(self.estop_flags & ESTOP_LATCHED)

    @property
    def active_causes(self) -> List[str]:
        return [n for bit, n in ESTOP_CAUSE_NAMES.items() if self.estop_flags & bit]

    @property
    def latch_causes(self) -> List[str]:
        return [n for bit, n in ESTOP_CAUSE_NAMES.items() if self.estop_latch_cause & bit]

    @property
    def vacuum_mode_name(self) -> str:
        return VACUUM_MODE_NAMES.get(self.vacuum_mode, "unknown")

    @property
    def vacuum_status_name(self) -> str:
        return VACUUM_STATUS_NAMES.get(self.vacuum_status & 0x7F, "unknown")

    @property
    def mic_running(self) -> bool:
        return bool(self.vacuum_status & VACUUM_STATUS_MIC_RUNNING)

    @property
    def yaw_rad(self) -> Optional[float]:
        return None if self.yaw_1e4_rad == YAW_NOT_AVAILABLE else self.yaw_1e4_rad * 1e-4

    def pack(self) -> bytes:
        return struct.pack(
            _STATUS_FMT, self.t_ms, self.left_ticks, self.right_ticks,
            *self.gyro_mrad_s, *self.accel_mm_s2, self.yaw_1e4_rad, self.bumper_bits,
            *self.cliff_mv, self.pack_mv, self.pack_ma, self.mic_level, self.estop_flags,
            self.estop_latch_cause, self.servo_angle_deg, self.vacuum_mode, self.vacuum_status,
            self.brush_duty_pct)

    @classmethod
    def unpack(cls, payload: bytes) -> "Status":
        if len(payload) != STATUS_SIZE:
            raise ValueError(f"status payload is {len(payload)} bytes, expected {STATUS_SIZE}")
        f = struct.unpack(_STATUS_FMT, payload)
        return cls(
            t_ms=f[0], left_ticks=f[1], right_ticks=f[2],
            gyro_mrad_s=(f[3], f[4], f[5]), accel_mm_s2=(f[6], f[7], f[8]),
            yaw_1e4_rad=f[9], bumper_bits=f[10], cliff_mv=(f[11], f[12], f[13], f[14]),
            pack_mv=f[15], pack_ma=f[16], mic_level=f[17], estop_flags=f[18],
            estop_latch_cause=f[19], servo_angle_deg=f[20], vacuum_mode=f[21],
            vacuum_status=f[22], brush_duty_pct=f[23])

    def to_dict(self) -> Dict:
        return {
            "t_ms": self.t_ms, "left_ticks": self.left_ticks, "right_ticks": self.right_ticks,
            "gyro_mrad_s": list(self.gyro_mrad_s), "accel_mm_s2": list(self.accel_mm_s2),
            "yaw_1e4_rad": self.yaw_1e4_rad, "bumper_bits": self.bumper_bits,
            "cliff_mv": list(self.cliff_mv), "pack_mv": self.pack_mv, "pack_ma": self.pack_ma,
            "mic_level": self.mic_level, "estop_flags": self.estop_flags,
            "estop_latch_cause": self.estop_latch_cause, "servo_angle_deg": self.servo_angle_deg,
            "vacuum_mode": self.vacuum_mode, "vacuum_status": self.vacuum_status,
            "brush_duty_pct": self.brush_duty_pct,
        }


@dataclass
class Ack:
    id: int = 0
    result: int = ACK_OK

    def pack(self) -> bytes:
        return struct.pack("<BB", self.id, self.result)

    @classmethod
    def unpack(cls, payload: bytes) -> "Ack":
        if len(payload) != 2:
            raise ValueError("ack payload must be 2 bytes")
        return cls(*struct.unpack("<BB", payload))


# ---- Generic field-level encode/decode (used by the vector test) ---------------
@dataclass
class Decoded:
    id: int
    fields: Dict = field(default_factory=dict)


def decode(msg_id: int, payload: bytes) -> Decoded:
    """Decode any message into a dict of named fields (same names as the vectors)."""
    if msg_id == SET_WHEEL_VEL:
        l, r = struct.unpack("<hh", payload)
        return Decoded(msg_id, {"left_mm_s": l, "right_mm_s": r})
    if msg_id == SET_SERVO:
        return Decoded(msg_id, {"angle_deg": payload[0]})
    if msg_id in (REQUEST_STATUS, CLEAR_ESTOP):
        if payload:
            raise ValueError("unexpected payload")
        return Decoded(msg_id, {})
    if msg_id == HEARTBEAT:
        return Decoded(msg_id, {"seq": payload[0]})
    if msg_id == SET_VACUUM_MODE:
        return Decoded(msg_id, {"mode": payload[0]})
    if msg_id == SET_BRUSH:
        return Decoded(msg_id, {"duty_pct": payload[0]})
    if msg_id == SET_STREAM:
        return Decoded(msg_id, {"enable": payload[0]})
    if msg_id == STATUS:
        return Decoded(msg_id, Status.unpack(payload).to_dict())
    if msg_id == ACK:
        a = Ack.unpack(payload)
        return Decoded(msg_id, {"id": a.id, "result": a.result})
    if msg_id == LOG:
        return Decoded(msg_id, {"text": payload.decode("ascii", errors="replace")})
    raise ValueError(f"unknown message id 0x{msg_id:02x}")


def encode(msg_id: int, fields: Dict) -> bytes:
    """Encode a whole frame from named fields (inverse of decode)."""
    if msg_id == SET_WHEEL_VEL:
        return set_wheel_vel(fields["left_mm_s"], fields["right_mm_s"])
    if msg_id == SET_SERVO:
        return set_servo(fields["angle_deg"])
    if msg_id == REQUEST_STATUS:
        return request_status()
    if msg_id == HEARTBEAT:
        return heartbeat(fields["seq"])
    if msg_id == CLEAR_ESTOP:
        return clear_estop()
    if msg_id == SET_VACUUM_MODE:
        return set_vacuum_mode(fields["mode"])
    if msg_id == SET_BRUSH:
        return set_brush(fields["duty_pct"])
    if msg_id == SET_STREAM:
        return set_stream(bool(fields["enable"]))
    if msg_id == STATUS:
        f = dict(fields)
        f["gyro_mrad_s"] = tuple(f["gyro_mrad_s"])
        f["accel_mm_s2"] = tuple(f["accel_mm_s2"])
        f["cliff_mv"] = tuple(f["cliff_mv"])
        return encode_frame(STATUS, Status(**f).pack())
    if msg_id == ACK:
        return encode_frame(ACK, Ack(fields["id"], fields["result"]).pack())
    if msg_id == LOG:
        return encode_frame(LOG, fields["text"].encode("ascii"))
    raise ValueError(f"unknown message id 0x{msg_id:02x}")
