#include "protocol.h"

namespace v8c {
namespace proto {

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000)
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            else
                crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t encode_frame(uint8_t id, const uint8_t* payload, size_t len, uint8_t* out, size_t out_cap) {
    if (len > kMaxPayload || out_cap < len + kOverhead) return 0;
    size_t i = 0;
    out[i++] = kSync0;
    out[i++] = kSync1;
    out[i++] = id;
    out[i++] = static_cast<uint8_t>(len);
    for (size_t k = 0; k < len; ++k) out[i++] = payload[k];
    const uint16_t crc = crc16_ccitt(&out[2], len + 2);
    out[i++] = static_cast<uint8_t>(crc & 0xFF);
    out[i++] = static_cast<uint8_t>(crc >> 8);
    return i;
}

void FrameParser::reset() {
    state_ = State::Sync0;
    pos_ = 0;
}

bool FrameParser::feed(uint8_t byte) {
    switch (state_) {
        case State::Sync0:
            if (byte == kSync0) state_ = State::Sync1;
            return false;
        case State::Sync1:
            if (byte == kSync1) {
                state_ = State::Id;
            } else {
                state_ = (byte == kSync0) ? State::Sync1 : State::Sync0;
            }
            return false;
        case State::Id:
            id_ = byte;
            state_ = State::Len;
            return false;
        case State::Len:
            if (byte > kMaxPayload) {
                state_ = State::Sync0;
                return false;
            }
            len_ = byte;
            pos_ = 0;
            state_ = (len_ == 0) ? State::Crc0 : State::Payload;
            return false;
        case State::Payload:
            payload_[pos_++] = byte;
            if (pos_ >= len_) state_ = State::Crc0;
            return false;
        case State::Crc0:
            crc_lo_ = byte;
            state_ = State::Crc1;
            return false;
        case State::Crc1: {
            state_ = State::Sync0;
            const uint16_t received = static_cast<uint16_t>(crc_lo_ | (byte << 8));
            uint16_t crc = 0xFFFF;
            const uint8_t head[2] = {id_, len_};
            crc = crc16_ccitt(head, 2, crc);
            crc = crc16_ccitt(payload_, len_, crc);
            if (crc == received) return true;
            ++crc_errors_;
            return false;
        }
    }
    return false;
}

// ---- Messages -------------------------------------------------------------

size_t SetWheelVelMsg::pack(uint8_t* out) const {
    size_t i = 0;
    put_i16(out, i, left_mm_s);
    put_i16(out, i, right_mm_s);
    return i;
}
bool SetWheelVelMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    size_t i = 0;
    left_mm_s = get_i16(in, i);
    right_mm_s = get_i16(in, i);
    return true;
}

size_t SetServoMsg::pack(uint8_t* out) const {
    out[0] = angle_deg;
    return kSize;
}
bool SetServoMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    angle_deg = in[0];
    return true;
}

size_t HeartbeatMsg::pack(uint8_t* out) const {
    out[0] = seq;
    return kSize;
}
bool HeartbeatMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    seq = in[0];
    return true;
}

size_t SetVacuumModeMsg::pack(uint8_t* out) const {
    out[0] = mode;
    return kSize;
}
bool SetVacuumModeMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    mode = in[0];
    return true;
}

size_t SetBrushMsg::pack(uint8_t* out) const {
    out[0] = duty_pct;
    return kSize;
}
bool SetBrushMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    duty_pct = in[0];
    return true;
}

size_t SetStreamMsg::pack(uint8_t* out) const {
    out[0] = enable;
    return kSize;
}
bool SetStreamMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    enable = in[0];
    return true;
}

size_t AckMsg::pack(uint8_t* out) const {
    out[0] = id;
    out[1] = result;
    return kSize;
}
bool AckMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    id = in[0];
    result = in[1];
    return true;
}

size_t StatusMsg::pack(uint8_t* out) const {
    size_t i = 0;
    put_u32(out, i, t_ms);
    put_i32(out, i, left_ticks);
    put_i32(out, i, right_ticks);
    for (int k = 0; k < 3; ++k) put_i16(out, i, gyro_mrad_s[k]);
    for (int k = 0; k < 3; ++k) put_i16(out, i, accel_mm_s2[k]);
    put_i16(out, i, yaw_1e4_rad);
    put_u8(out, i, bumper_bits);
    for (int k = 0; k < 4; ++k) put_u16(out, i, cliff_mv[k]);
    put_u16(out, i, pack_mv);
    put_i16(out, i, pack_ma);
    put_u16(out, i, mic_level);
    put_u8(out, i, estop_flags);
    put_u8(out, i, estop_latch_cause);
    put_u8(out, i, servo_angle_deg);
    put_u8(out, i, vacuum_mode);
    put_u8(out, i, vacuum_status);
    put_u8(out, i, brush_duty_pct);
    return i;  // == kSize
}

bool StatusMsg::unpack(const uint8_t* in, size_t len) {
    if (len != kSize) return false;
    size_t i = 0;
    t_ms = get_u32(in, i);
    left_ticks = get_i32(in, i);
    right_ticks = get_i32(in, i);
    for (int k = 0; k < 3; ++k) gyro_mrad_s[k] = get_i16(in, i);
    for (int k = 0; k < 3; ++k) accel_mm_s2[k] = get_i16(in, i);
    yaw_1e4_rad = get_i16(in, i);
    bumper_bits = get_u8(in, i);
    for (int k = 0; k < 4; ++k) cliff_mv[k] = get_u16(in, i);
    pack_mv = get_u16(in, i);
    pack_ma = get_i16(in, i);
    mic_level = get_u16(in, i);
    estop_flags = get_u8(in, i);
    estop_latch_cause = get_u8(in, i);
    servo_angle_deg = get_u8(in, i);
    vacuum_mode = get_u8(in, i);
    vacuum_status = get_u8(in, i);
    brush_duty_pct = get_u8(in, i);
    return i == kSize;
}

}  // namespace proto
}  // namespace v8c
