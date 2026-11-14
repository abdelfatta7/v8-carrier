// IMU behind a small interface so the register-level driver can be swapped:
// Bno085Imu (default, through the Adafruit BNO08x library when it is in
// lib_deps) or Mpu6050Imu (raw I2C registers, no library). main.cpp picks one
// with IMU_DRIVER in platformio.ini. Hardware layer (Arduino).
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

struct ImuSample {
    float gyro[3] = {0, 0, 0};   // rad/s, x y z, body frame (x forward, y left, z up)
    float accel[3] = {0, 0, 0};  // m/s^2 including gravity
    float yaw = NAN;             // rad, fused heading if the sensor provides one
};

class ImuDriver {
public:
    virtual ~ImuDriver() = default;
    virtual bool begin(TwoWire& wire) = 0;
    // Fill `out` with the latest data. Returns false when nothing new.
    virtual bool read(ImuSample& out) = 0;
    virtual const char* name() const = 0;
};

// ---- No IMU fitted / library missing: reports nothing --------------------
class NullImu : public ImuDriver {
public:
    bool begin(TwoWire&) override { return false; }
    bool read(ImuSample&) override { return false; }
    const char* name() const override { return "none"; }
};

// ---- BNO085 via Adafruit_BNO08x (SH-2 protocol is too big to hand-roll) ----
#if __has_include(<Adafruit_BNO08x.h>)
#include <Adafruit_BNO08x.h>

class Bno085Imu : public ImuDriver {
public:
    explicit Bno085Imu(uint8_t addr = 0x4A) : addr_(addr) {}

    bool begin(TwoWire& wire) override {
        if (!bno_.begin_I2C(addr_, &wire)) return false;
        ok_ = true;
        enable_reports();
        return true;
    }

    bool read(ImuSample& out) override {
        if (!ok_) return false;
        if (bno_.wasReset()) enable_reports();
        bool any = false;
        sh2_SensorValue_t v;
        while (bno_.getSensorEvent(&v)) {
            any = true;
            switch (v.sensorId) {
                case SH2_GYROSCOPE_CALIBRATED:
                    last_.gyro[0] = v.un.gyroscope.x;
                    last_.gyro[1] = v.un.gyroscope.y;
                    last_.gyro[2] = v.un.gyroscope.z;
                    break;
                case SH2_ACCELEROMETER:
                    last_.accel[0] = v.un.accelerometer.x;
                    last_.accel[1] = v.un.accelerometer.y;
                    last_.accel[2] = v.un.accelerometer.z;
                    break;
                case SH2_GAME_ROTATION_VECTOR: {
                    // Yaw from the quaternion (no magnetometer: drift-free
                    // relative heading, which is what the EKF wants).
                    const float qi = v.un.gameRotationVector.i, qj = v.un.gameRotationVector.j;
                    const float qk = v.un.gameRotationVector.k, qr = v.un.gameRotationVector.real;
                    last_.yaw = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (qj * qj + qk * qk));
                    break;
                }
                default:
                    break;
            }
        }
        if (any) out = last_;
        return any;
    }

    const char* name() const override { return "BNO085"; }

private:
    void enable_reports() {
        bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);   // 100 Hz
        bno_.enableReport(SH2_ACCELEROMETER, 10000);
        bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    }
    uint8_t addr_;
    Adafruit_BNO08x bno_{-1};
    bool ok_ = false;
    ImuSample last_;
};
#else
// Library not available: keep the symbol so main.cpp compiles, report nothing.
class Bno085Imu : public NullImu {
public:
    explicit Bno085Imu(uint8_t = 0x4A) {}
    const char* name() const override { return "BNO085 (library missing)"; }
};
#endif

// ---- MPU-6050, raw registers ----------------------------------------------
class Mpu6050Imu : public ImuDriver {
public:
    explicit Mpu6050Imu(uint8_t addr = 0x68) : addr_(addr) {}

    bool begin(TwoWire& wire) override {
        wire_ = &wire;
        if (!write_reg(0x6B, 0x01)) return false;  // PWR_MGMT_1: wake, PLL with X gyro
        write_reg(0x1A, 0x03);                      // CONFIG: DLPF 44 Hz
        write_reg(0x1B, 0x00);                      // GYRO_CONFIG: +-250 deg/s
        write_reg(0x1C, 0x00);                      // ACCEL_CONFIG: +-2 g
        ok_ = true;
        return true;
    }

    bool read(ImuSample& out) override {
        if (!ok_) return false;
        wire_->beginTransmission(addr_);
        wire_->write(0x3B);  // ACCEL_XOUT_H
        if (wire_->endTransmission(false) != 0) return false;
        if (wire_->requestFrom(static_cast<int>(addr_), 14) != 14) return false;
        int16_t raw[7];
        for (int i = 0; i < 7; ++i) {
            const int hi = wire_->read(), lo = wire_->read();
            raw[i] = static_cast<int16_t>((hi << 8) | lo);
        }
        const float g = 9.80665f;
        for (int i = 0; i < 3; ++i) out.accel[i] = raw[i] / 16384.0f * g;
        for (int i = 0; i < 3; ++i) out.gyro[i] = raw[4 + i] / 131.0f * (3.14159265f / 180.0f);
        out.yaw = NAN;  // no fusion on chip; the Pi's EKF integrates the rate
        return true;
    }

    const char* name() const override { return "MPU-6050"; }

private:
    bool write_reg(uint8_t reg, uint8_t val) {
        wire_->beginTransmission(addr_);
        wire_->write(reg);
        wire_->write(val);
        return wire_->endTransmission() == 0;
    }
    uint8_t addr_;
    TwoWire* wire_ = nullptr;
    bool ok_ = false;
};
