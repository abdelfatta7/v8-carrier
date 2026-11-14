// Protocol tests. When run with a file path argument the test also writes the
// test vectors (docs/protocol_vectors.json) that the Python decoder on the Pi
// side is checked against, so both sides prove they share one byte layout.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../minitest.h"
#include "protocol.h"

using namespace v8c::proto;

static std::string hex(const uint8_t* p, size_t n) {
    static const char* d = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; ++i) {
        s += d[p[i] >> 4];
        s += d[p[i] & 0xF];
    }
    return s;
}

TEST(crc16_ccitt_false_check_value) {
    const uint8_t s[] = "123456789";
    CHECK_EQ(crc16_ccitt(s, 9), 0x29B1);
    CHECK_EQ(crc16_ccitt(nullptr, 0), 0xFFFF);
}

TEST(encode_frame_layout) {
    uint8_t payload[2] = {0x12, 0x34};
    uint8_t out[16];
    const size_t n = encode_frame(0x01, payload, 2, out, sizeof out);
    CHECK_EQ(n, 8u);
    CHECK_EQ(out[0], 0xAA);
    CHECK_EQ(out[1], 0x55);
    CHECK_EQ(out[2], 0x01);
    CHECK_EQ(out[3], 0x02);
    CHECK_EQ(out[4], 0x12);
    CHECK_EQ(out[5], 0x34);
    const uint8_t covered[] = {0x01, 0x02, 0x12, 0x34};
    const uint16_t crc = crc16_ccitt(covered, 4);
    CHECK_EQ(out[6], crc & 0xFF);
    CHECK_EQ(out[7], crc >> 8);
    // Too small a buffer or too large a payload is refused.
    CHECK_EQ(encode_frame(0x01, payload, 2, out, 7), 0u);
    uint8_t big[kMaxPayload + 1] = {};
    uint8_t out2[kMaxFrame + 8];
    CHECK_EQ(encode_frame(0x01, big, kMaxPayload + 1, out2, sizeof out2), 0u);
}

TEST(parser_round_trip_and_resync) {
    uint8_t payload[4] = {1, 2, 3, 4};
    uint8_t frame[16];
    const size_t n = encode_frame(SET_WHEEL_VEL, payload, 4, frame, sizeof frame);
    FrameParser p;
    // Garbage, then a stray sync byte, then a real frame, then a corrupted copy.
    const uint8_t noise[] = {0x00, 0xFF, 0xAA, 0x12, 0xAA};
    for (uint8_t b : noise) CHECK(!p.feed(b));
    int got = 0;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(frame[i])) ++got;
    CHECK_EQ(got, 1);
    CHECK_EQ(p.id(), SET_WHEEL_VEL);
    CHECK_EQ(p.length(), 4u);
    CHECK_EQ(std::memcmp(p.payload(), payload, 4), 0);

    frame[5] ^= 0x01;  // corrupt a payload byte
    got = 0;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(frame[i])) ++got;
    CHECK_EQ(got, 0);
    CHECK_EQ(p.crc_errors(), 1u);

    // Parser still works after the error.
    frame[5] ^= 0x01;
    got = 0;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(frame[i])) ++got;
    CHECK_EQ(got, 1);
}

TEST(parser_handles_empty_payload_and_sync_inside_payload) {
    uint8_t frame[16];
    const size_t n = encode_frame(REQUEST_STATUS, nullptr, 0, frame, sizeof frame);
    CHECK_EQ(n, 6u);
    FrameParser p;
    int got = 0;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(frame[i])) ++got;
    CHECK_EQ(got, 1);
    CHECK_EQ(p.length(), 0u);

    // Payload that contains the sync pattern must not confuse the parser.
    uint8_t payload[4] = {0xAA, 0x55, 0xAA, 0x55};
    const size_t m = encode_frame(LOG, payload, 4, frame, sizeof frame);
    got = 0;
    for (size_t i = 0; i < m; ++i)
        if (p.feed(frame[i])) ++got;
    CHECK_EQ(got, 1);
    CHECK_EQ(p.id(), LOG);
}

TEST(status_message_round_trip) {
    StatusMsg s;
    s.t_ms = 123456789u;
    s.left_ticks = -12345;
    s.right_ticks = 2000000000;
    s.gyro_mrad_s[0] = -1;
    s.gyro_mrad_s[1] = 2;
    s.gyro_mrad_s[2] = -3000;
    s.accel_mm_s2[0] = 100;
    s.accel_mm_s2[1] = -200;
    s.accel_mm_s2[2] = 9810;
    s.yaw_1e4_rad = -31416;
    s.bumper_bits = 0x03;
    s.cliff_mv[0] = 1900;
    s.cliff_mv[1] = 2000;
    s.cliff_mv[2] = 450;
    s.cliff_mv[3] = 2100;
    s.pack_mv = 15800;
    s.pack_ma = -1200;
    s.mic_level = 4321;
    s.estop_flags = ESTOP_LATCHED | ESTOP_CLIFF;
    s.estop_latch_cause = ESTOP_CLIFF;
    s.servo_angle_deg = 20;
    s.vacuum_mode = VAC_SOFT;
    s.vacuum_status = VAC_IDLE | kVacuumStatusMicRunning;
    s.brush_duty_pct = 80;

    uint8_t buf[64];
    const size_t n = s.pack(buf);
    CHECK_EQ(n, StatusMsg::kSize);
    StatusMsg d;
    CHECK(d.unpack(buf, n));
    CHECK(!d.unpack(buf, n - 1));
    CHECK_EQ(d.t_ms, s.t_ms);
    CHECK_EQ(d.left_ticks, s.left_ticks);
    CHECK_EQ(d.right_ticks, s.right_ticks);
    for (int k = 0; k < 3; ++k) CHECK_EQ(d.gyro_mrad_s[k], s.gyro_mrad_s[k]);
    for (int k = 0; k < 3; ++k) CHECK_EQ(d.accel_mm_s2[k], s.accel_mm_s2[k]);
    CHECK_EQ(d.yaw_1e4_rad, s.yaw_1e4_rad);
    CHECK_EQ(d.bumper_bits, s.bumper_bits);
    for (int k = 0; k < 4; ++k) CHECK_EQ(d.cliff_mv[k], s.cliff_mv[k]);
    CHECK_EQ(d.pack_mv, s.pack_mv);
    CHECK_EQ(d.pack_ma, s.pack_ma);
    CHECK_EQ(d.mic_level, s.mic_level);
    CHECK_EQ(d.estop_flags, s.estop_flags);
    CHECK_EQ(d.estop_latch_cause, s.estop_latch_cause);
    CHECK_EQ(d.servo_angle_deg, s.servo_angle_deg);
    CHECK_EQ(d.vacuum_mode, s.vacuum_mode);
    CHECK_EQ(d.vacuum_status, s.vacuum_status);
    CHECK_EQ(d.brush_duty_pct, s.brush_duty_pct);
    // Little-endian spot check of the first field.
    CHECK_EQ(buf[0], 0x15);
    CHECK_EQ(buf[1], 0xCD);
    CHECK_EQ(buf[2], 0x5B);
    CHECK_EQ(buf[3], 0x07);
}

TEST(small_messages_round_trip) {
    uint8_t buf[8];
    SetWheelVelMsg v;
    v.left_mm_s = -300;
    v.right_mm_s = 250;
    CHECK_EQ(v.pack(buf), 4u);
    SetWheelVelMsg v2;
    CHECK(v2.unpack(buf, 4));
    CHECK_EQ(v2.left_mm_s, -300);
    CHECK_EQ(v2.right_mm_s, 250);
    CHECK(!v2.unpack(buf, 3));

    AckMsg a;
    a.id = SET_SERVO;
    a.result = ACK_REJECTED_ESTOP;
    CHECK_EQ(a.pack(buf), 2u);
    AckMsg a2;
    CHECK(a2.unpack(buf, 2));
    CHECK_EQ(a2.id, SET_SERVO);
    CHECK_EQ(a2.result, ACK_REJECTED_ESTOP);

    HeartbeatMsg h;
    h.seq = 200;
    h.pack(buf);
    HeartbeatMsg h2;
    CHECK(h2.unpack(buf, 1));
    CHECK_EQ(h2.seq, 200);
}

// ---- Test vector generation -----------------------------------------------

struct Vector {
    std::string name;
    uint8_t id;
    std::string fields_json;  // already formatted JSON object body
    std::vector<uint8_t> frame;
};

static std::vector<Vector> make_vectors() {
    std::vector<Vector> v;
    uint8_t payload[64];
    uint8_t frame[80];
    auto push = [&](const char* name, uint8_t id, size_t len, const std::string& fields) {
        const size_t n = encode_frame(id, payload, len, frame, sizeof frame);
        v.push_back({name, id, fields, std::vector<uint8_t>(frame, frame + n)});
    };

    {
        SetWheelVelMsg m;
        m.left_mm_s = 150;
        m.right_mm_s = -150;
        push("set_wheel_vel", SET_WHEEL_VEL, m.pack(payload), "{\"left_mm_s\": 150, \"right_mm_s\": -150}");
    }
    {
        SetServoMsg m;
        m.angle_deg = 70;
        push("set_servo", SET_SERVO, m.pack(payload), "{\"angle_deg\": 70}");
    }
    push("request_status", REQUEST_STATUS, 0, "{}");
    {
        HeartbeatMsg m;
        m.seq = 7;
        push("heartbeat", HEARTBEAT, m.pack(payload), "{\"seq\": 7}");
    }
    push("clear_estop", CLEAR_ESTOP, 0, "{}");
    {
        SetVacuumModeMsg m;
        m.mode = VAC_HARD;
        push("set_vacuum_mode", SET_VACUUM_MODE, m.pack(payload), "{\"mode\": 2}");
    }
    {
        SetBrushMsg m;
        m.duty_pct = 100;
        push("set_brush", SET_BRUSH, m.pack(payload), "{\"duty_pct\": 100}");
    }
    {
        SetStreamMsg m;
        m.enable = 1;
        push("set_stream", SET_STREAM, m.pack(payload), "{\"enable\": 1}");
    }
    {
        AckMsg m;
        m.id = SET_WHEEL_VEL;
        m.result = ACK_REJECTED_ESTOP;
        push("ack", ACK, m.pack(payload), "{\"id\": 1, \"result\": 1}");
    }
    {
        const char* text = "hello";
        std::memcpy(payload, text, 5);
        push("log", LOG, 5, "{\"text\": \"hello\"}");
    }
    {
        StatusMsg s;
        s.t_ms = 123456789u;
        s.left_ticks = -12345;
        s.right_ticks = 2000000000;
        s.gyro_mrad_s[0] = -1;
        s.gyro_mrad_s[1] = 2;
        s.gyro_mrad_s[2] = -3000;
        s.accel_mm_s2[0] = 100;
        s.accel_mm_s2[1] = -200;
        s.accel_mm_s2[2] = 9810;
        s.yaw_1e4_rad = -31416;
        s.bumper_bits = 3;
        s.cliff_mv[0] = 1900;
        s.cliff_mv[1] = 2000;
        s.cliff_mv[2] = 450;
        s.cliff_mv[3] = 2100;
        s.pack_mv = 15800;
        s.pack_ma = -1200;
        s.mic_level = 4321;
        s.estop_flags = ESTOP_LATCHED | ESTOP_CLIFF;
        s.estop_latch_cause = ESTOP_CLIFF;
        s.servo_angle_deg = 20;
        s.vacuum_mode = VAC_SOFT;
        s.vacuum_status = VAC_IDLE | kVacuumStatusMicRunning;
        s.brush_duty_pct = 80;
        push("status", STATUS, s.pack(payload),
             "{\"t_ms\": 123456789, \"left_ticks\": -12345, \"right_ticks\": 2000000000, "
             "\"gyro_mrad_s\": [-1, 2, -3000], \"accel_mm_s2\": [100, -200, 9810], "
             "\"yaw_1e4_rad\": -31416, \"bumper_bits\": 3, \"cliff_mv\": [1900, 2000, 450, 2100], "
             "\"pack_mv\": 15800, \"pack_ma\": -1200, \"mic_level\": 4321, \"estop_flags\": 5, "
             "\"estop_latch_cause\": 4, \"servo_angle_deg\": 20, \"vacuum_mode\": 1, "
             "\"vacuum_status\": 128, \"brush_duty_pct\": 80}");
    }
    return v;
}

static bool write_vectors(const char* path) {
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    std::fprintf(f, "{\n");
    std::fprintf(f, "  \"_comment\": \"Generated by firmware/test/test_protocol; do not edit by hand. "
                    "Regenerate with: cmake --build firmware/build --target update_protocol_vectors\",\n");
    std::fprintf(f, "  \"sync\": [170, 85],\n");
    std::fprintf(f, "  \"max_payload\": %u,\n", (unsigned)kMaxPayload);
    std::fprintf(f, "  \"crc\": {\"name\": \"CRC-16/CCITT-FALSE\", \"poly\": 4129, \"init\": 65535, "
                    "\"check_123456789\": 10673},\n");
    std::fprintf(f, "  \"status_size\": %u,\n", (unsigned)StatusMsg::kSize);
    std::fprintf(f, "  \"frames\": [\n");
    const auto vectors = make_vectors();
    for (size_t i = 0; i < vectors.size(); ++i) {
        const Vector& v = vectors[i];
        std::fprintf(f, "    {\"name\": \"%s\", \"id\": %u, \"fields\": %s, \"frame_hex\": \"%s\"}%s\n",
                     v.name.c_str(), (unsigned)v.id, v.fields_json.c_str(), hex(v.frame.data(), v.frame.size()).c_str(),
                     (i + 1 < vectors.size()) ? "," : "");
    }
    std::fprintf(f, "  ]\n}\n");
    std::fclose(f);
    return true;
}

TEST(vectors_parse_back) {
    for (const Vector& v : make_vectors()) {
        FrameParser p;
        int got = 0;
        for (uint8_t b : v.frame)
            if (p.feed(b)) ++got;
        CHECK_EQ(got, 1);
        CHECK_EQ(p.id(), v.id);
    }
}

int main(int argc, char** argv) {
    const int failed = minitest::run_all("protocol");
    if (argc > 1) {
        if (!write_vectors(argv[1])) {
            std::printf("could not write %s\n", argv[1]);
            return 1;
        }
        std::printf("wrote %s\n", argv[1]);
    }
    return failed;
}
