// Serial protocol between the Pi and the ESP32-S3. The authoritative
// description is docs/protocol.md; this header and the Python mirror in
// ros2_ws/src/v8carrier_bringup/v8carrier_bringup/protocol.py must agree with
// it byte for byte (docs/protocol_vectors.json, written by test_protocol, is
// the cross-check).
//
// Frame:  0xAA 0x55 | id u8 | len u8 | payload[len] | crc16 lo | crc16 hi
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out)
// over id, len and payload. Multi-byte fields are little-endian.
#pragma once

#include <cstddef>
#include <cstdint>

namespace v8c {
namespace proto {

constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;
constexpr size_t kMaxPayload = 64;
constexpr size_t kOverhead = 6;  // 2 sync + id + len + crc16
constexpr size_t kMaxFrame = kMaxPayload + kOverhead;

// Message ids. Pi -> ESP32 are < 0x80, ESP32 -> Pi are >= 0x80.
enum MsgId : uint8_t {
    SET_WHEEL_VEL = 0x01,    // i16 left_mm_s, i16 right_mm_s
    SET_SERVO = 0x02,        // u8 angle_deg (0..180)
    REQUEST_STATUS = 0x03,   // (none)
    HEARTBEAT = 0x04,        // u8 seq
    CLEAR_ESTOP = 0x05,      // (none)
    SET_VACUUM_MODE = 0x06,  // u8 mode (0 off, 1 soft, 2 hard)
    SET_BRUSH = 0x07,        // u8 duty_pct (0..100)
    SET_STREAM = 0x08,       // u8 enable (0/1)

    STATUS = 0x80,  // see StatusMsg
    ACK = 0x81,     // u8 id, u8 result
    LOG = 0x82,     // ASCII text, not NUL-terminated
};

enum AckResult : uint8_t {
    ACK_OK = 0,
    ACK_REJECTED_ESTOP = 1,  // command refused because the estop is latched
    ACK_BAD_ARG = 2,         // payload length or value out of range
    ACK_BUSY = 3,            // e.g. vacuum button routine already running
};

// Bits of StatusMsg::estop_flags and estop_latch_cause.
enum EstopBits : uint8_t {
    ESTOP_LATCHED = 1u << 0,    // motors cut; only in estop_flags
    ESTOP_BUMPER = 1u << 1,
    ESTOP_CLIFF = 1u << 2,
    ESTOP_LOW_BATTERY = 1u << 3,
    ESTOP_HEARTBEAT = 1u << 4,
};

enum VacuumMode : uint8_t { VAC_OFF = 0, VAC_SOFT = 1, VAC_HARD = 2, VAC_UNKNOWN = 3 };
enum VacuumStatus : uint8_t { VAC_IDLE = 0, VAC_BUSY = 1, VAC_FAULT = 2 };
constexpr uint8_t kVacuumStatusMicRunning = 1u << 7;  // OR-ed into vacuum_status

// ---- CRC ------------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// ---- Framing --------------------------------------------------------------
// Writes a complete frame into out (capacity >= len + kOverhead). Returns the
// frame size, or 0 if len > kMaxPayload or out is too small.
size_t encode_frame(uint8_t id, const uint8_t* payload, size_t len, uint8_t* out, size_t out_cap);

// Byte-at-a-time frame parser. feed() returns true exactly once per complete,
// CRC-valid frame; the frame stays readable until the next feed() call.
class FrameParser {
public:
    bool feed(uint8_t byte);
    uint8_t id() const { return id_; }
    const uint8_t* payload() const { return payload_; }
    size_t length() const { return len_; }
    uint32_t crc_errors() const { return crc_errors_; }
    void reset();

private:
    enum class State : uint8_t { Sync0, Sync1, Id, Len, Payload, Crc0, Crc1 };
    State state_ = State::Sync0;
    uint8_t id_ = 0;
    uint8_t len_ = 0;
    uint8_t pos_ = 0;
    uint8_t payload_[kMaxPayload] = {};
    uint8_t crc_lo_ = 0;
    uint32_t crc_errors_ = 0;
};

// ---- Little-endian field helpers ------------------------------------------
inline void put_u8(uint8_t* p, size_t& i, uint8_t v) { p[i++] = v; }
inline void put_u16(uint8_t* p, size_t& i, uint16_t v) {
    p[i++] = static_cast<uint8_t>(v & 0xFF);
    p[i++] = static_cast<uint8_t>(v >> 8);
}
inline void put_i16(uint8_t* p, size_t& i, int16_t v) { put_u16(p, i, static_cast<uint16_t>(v)); }
inline void put_u32(uint8_t* p, size_t& i, uint32_t v) {
    put_u16(p, i, static_cast<uint16_t>(v & 0xFFFF));
    put_u16(p, i, static_cast<uint16_t>(v >> 16));
}
inline void put_i32(uint8_t* p, size_t& i, int32_t v) { put_u32(p, i, static_cast<uint32_t>(v)); }

inline uint8_t get_u8(const uint8_t* p, size_t& i) { return p[i++]; }
inline uint16_t get_u16(const uint8_t* p, size_t& i) {
    uint16_t v = static_cast<uint16_t>(p[i] | (p[i + 1] << 8));
    i += 2;
    return v;
}
inline int16_t get_i16(const uint8_t* p, size_t& i) { return static_cast<int16_t>(get_u16(p, i)); }
inline uint32_t get_u32(const uint8_t* p, size_t& i) {
    uint32_t lo = get_u16(p, i);
    uint32_t hi = get_u16(p, i);
    return lo | (hi << 16);
}
inline int32_t get_i32(const uint8_t* p, size_t& i) { return static_cast<int32_t>(get_u32(p, i)); }

// ---- Messages -------------------------------------------------------------
// Each message has a fixed payload size, a pack() that writes the payload and
// an unpack() that returns false when the length does not match.

struct SetWheelVelMsg {
    int16_t left_mm_s = 0;
    int16_t right_mm_s = 0;
    static constexpr size_t kSize = 4;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct SetServoMsg {
    uint8_t angle_deg = 0;
    static constexpr size_t kSize = 1;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct HeartbeatMsg {
    uint8_t seq = 0;
    static constexpr size_t kSize = 1;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct SetVacuumModeMsg {
    uint8_t mode = VAC_OFF;
    static constexpr size_t kSize = 1;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct SetBrushMsg {
    uint8_t duty_pct = 0;
    static constexpr size_t kSize = 1;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct SetStreamMsg {
    uint8_t enable = 1;
    static constexpr size_t kSize = 1;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

struct AckMsg {
    uint8_t id = 0;
    uint8_t result = ACK_OK;
    static constexpr size_t kSize = 2;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

// Telemetry sent at 50 Hz. Units are chosen so every field fits an integer:
// mrad/s, mm/s^2, 1e-4 rad, mV, mA. 47 bytes.
struct StatusMsg {
    uint32_t t_ms = 0;            // ESP32 millis()
    int32_t left_ticks = 0;       // cumulative encoder ticks, left wheel
    int32_t right_ticks = 0;      // cumulative encoder ticks, right wheel
    int16_t gyro_mrad_s[3] = {0, 0, 0};    // x, y, z angular rate
    int16_t accel_mm_s2[3] = {0, 0, 0};    // x, y, z linear acceleration (incl. gravity)
    int16_t yaw_1e4_rad = 0;      // fused heading from the IMU, 0x7FFF = not available
    uint8_t bumper_bits = 0;      // bit0 left, bit1 right (1 = pressed)
    uint16_t cliff_mv[4] = {0, 0, 0, 0};  // nose-left, nose-right, wheel-left, wheel-right
    uint16_t pack_mv = 0;         // pack voltage
    int16_t pack_ma = 0;          // pack current, positive = discharging
    uint16_t mic_level = 0;       // RMS of the microphone window (16-bit sample units)
    uint8_t estop_flags = 0;      // EstopBits (bit0 latched + currently active causes)
    uint8_t estop_latch_cause = 0;  // causes that were active when the latch set
    uint8_t servo_angle_deg = 0;  // commanded servo angle
    uint8_t vacuum_mode = VAC_UNKNOWN;    // VacuumMode
    uint8_t vacuum_status = VAC_IDLE;     // VacuumStatus | kVacuumStatusMicRunning
    uint8_t brush_duty_pct = 0;
    static constexpr size_t kSize = 47;
    size_t pack(uint8_t* out) const;
    bool unpack(const uint8_t* in, size_t len);
};

}  // namespace proto
}  // namespace v8c
