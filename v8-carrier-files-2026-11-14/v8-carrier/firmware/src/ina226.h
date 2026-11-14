// INA226 pack voltage / current monitor, raw registers. Hardware layer (Arduino).
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Ina226 {
public:
    // shunt_ohms: the shunt on the breakout; current_lsb_a: resolution (1 mA).
    bool begin(TwoWire& wire, uint8_t addr, float shunt_ohms, float current_lsb_a = 0.001f) {
        wire_ = &wire;
        addr_ = addr;
        current_lsb_a_ = current_lsb_a;
        // CONFIG: average 16 samples, 1.1 ms conversions, continuous shunt+bus.
        if (!write16(0x00, 0x4527)) return false;
        const float cal = 0.00512f / (current_lsb_a * shunt_ohms);
        write16(0x05, static_cast<uint16_t>(cal));
        ok_ = true;
        return true;
    }

    bool ok() const { return ok_; }

    // Bus voltage in volts (LSB 1.25 mV).
    bool read_voltage(float& v) {
        uint16_t raw;
        if (!read16(0x02, raw)) return false;
        v = raw * 1.25e-3f;
        return true;
    }

    // Current in amps, positive = flowing from IN+ to IN- (discharging).
    bool read_current(float& a) {
        uint16_t raw;
        if (!read16(0x04, raw)) return false;
        a = static_cast<int16_t>(raw) * current_lsb_a_;
        return true;
    }

private:
    bool write16(uint8_t reg, uint16_t val) {
        wire_->beginTransmission(addr_);
        wire_->write(reg);
        wire_->write(static_cast<uint8_t>(val >> 8));
        wire_->write(static_cast<uint8_t>(val & 0xFF));
        return wire_->endTransmission() == 0;
    }
    bool read16(uint8_t reg, uint16_t& val) {
        wire_->beginTransmission(addr_);
        wire_->write(reg);
        if (wire_->endTransmission(false) != 0) return false;
        if (wire_->requestFrom(static_cast<int>(addr_), 2) != 2) return false;
        const int hi = wire_->read(), lo = wire_->read();
        val = static_cast<uint16_t>((hi << 8) | lo);
        return true;
    }
    TwoWire* wire_ = nullptr;
    uint8_t addr_ = 0x40;
    float current_lsb_a_ = 0.001f;
    bool ok_ = false;
};
