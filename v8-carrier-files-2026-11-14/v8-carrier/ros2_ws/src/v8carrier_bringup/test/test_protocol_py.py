"""The Python protocol mirror must decode and re-encode exactly the frames the
C++ encoder wrote into docs/protocol_vectors.json."""

import json
import os

import pytest

from v8carrier_bringup import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
VECTORS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "docs", "protocol_vectors.json"))


@pytest.fixture(scope="module")
def vectors():
    with open(VECTORS) as f:
        return json.load(f)


def test_crc_check_value(vectors):
    assert P.crc16_ccitt(b"123456789") == 0x29B1
    assert P.crc16_ccitt(b"123456789") == vectors["crc"]["check_123456789"]
    assert P.crc16_ccitt(b"") == 0xFFFF


def test_constants_match_vectors(vectors):
    assert [P.SYNC0, P.SYNC1] == vectors["sync"]
    assert P.MAX_PAYLOAD == vectors["max_payload"]
    assert P.STATUS_SIZE == vectors["status_size"]


def test_every_vector_decodes_to_the_same_fields(vectors):
    for v in vectors["frames"]:
        frame = bytes.fromhex(v["frame_hex"])
        parser = P.FrameParser()
        frames = parser.feed(frame)
        assert len(frames) == 1, v["name"]
        msg_id, payload = frames[0]
        assert msg_id == v["id"], v["name"]
        decoded = P.decode(msg_id, payload)
        assert decoded.fields == v["fields"], v["name"]
        assert parser.crc_errors == 0


def test_every_vector_re_encodes_to_the_same_bytes(vectors):
    for v in vectors["frames"]:
        assert P.encode(v["id"], v["fields"]).hex() == v["frame_hex"], v["name"]


def test_parser_resyncs_after_garbage_and_bad_crc(vectors):
    good = bytes.fromhex(vectors["frames"][0]["frame_hex"])
    bad = bytearray(good)
    bad[5] ^= 0x01
    stream = b"\x00\xff\xaa\x12" + bytes(bad) + b"\xaa" + good + good[:3]
    parser = P.FrameParser()
    frames = parser.feed(stream)
    assert len(frames) == 1
    assert parser.crc_errors == 1
    # The rest of the split frame arrives later.
    frames = parser.feed(good[3:])
    assert len(frames) == 1


def test_parser_handles_byte_at_a_time(vectors):
    status = bytes.fromhex(next(v for v in vectors["frames"] if v["name"] == "status")["frame_hex"])
    parser = P.FrameParser()
    got = []
    for b in status:
        got += parser.feed(bytes([b]))
    assert len(got) == 1
    s = P.Status.unpack(got[0][1])
    assert s.estop_latched
    assert s.active_causes == ["cliff"]
    assert s.latch_causes == ["cliff"]
    assert s.vacuum_mode_name == "soft"
    assert s.vacuum_status_name == "idle"
    assert s.mic_running
    assert s.yaw_rad == pytest.approx(-3.1416)


def test_status_pack_unpack_round_trip():
    s = P.Status(t_ms=1, left_ticks=-5, right_ticks=5, gyro_mrad_s=(1, 2, 3), accel_mm_s2=(4, 5, 6),
                 yaw_1e4_rad=P.YAW_NOT_AVAILABLE, bumper_bits=1, cliff_mv=(1, 2, 3, 4), pack_mv=16000,
                 pack_ma=-100, mic_level=7, estop_flags=0, estop_latch_cause=0, servo_angle_deg=20,
                 vacuum_mode=P.VAC_OFF, vacuum_status=P.VAC_FAULT, brush_duty_pct=0)
    assert P.Status.unpack(s.pack()) == s
    assert s.yaw_rad is None
    assert not s.estop_latched
    with pytest.raises(ValueError):
        P.Status.unpack(s.pack()[:-1])


def test_command_encoders_clamp_and_validate():
    assert P.set_wheel_vel(40000, -40000)[4:8] == b"\xff\x7f\x00\x80"
    assert P.set_servo(300)[4] == 180
    assert P.set_brush(150)[4] == 100
    with pytest.raises(ValueError):
        P.set_vacuum_mode(3)
    with pytest.raises(ValueError):
        P.encode_frame(P.LOG, b"x" * 65)
