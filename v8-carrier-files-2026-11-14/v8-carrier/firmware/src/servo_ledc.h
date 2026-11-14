// Hobby servo on LEDC at 50 Hz, 16-bit. Hardware layer (Arduino).
#pragma once

#include <Arduino.h>

class ServoLedc {
public:
    void begin(int pin, int channel, uint16_t min_us, uint16_t max_us) {
        pin_ = pin;
        min_us_ = min_us;
        max_us_ = max_us;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        (void)channel;
        ledcAttach(pin_, 50, 16);
#else
        ch_ = channel;
        ledcSetup(ch_, 50, 16);
        ledcAttachPin(pin_, ch_);
#endif
    }

    void write_deg(uint8_t deg) {
        if (deg > 180) deg = 180;
        if (deg == last_deg_) return;
        last_deg_ = deg;
        const uint32_t us = min_us_ + static_cast<uint32_t>(max_us_ - min_us_) * deg / 180u;
        const uint32_t duty = us * 65535u / 20000u;  // 20 ms period
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(pin_, duty);
#else
        ledcWrite(ch_, duty);
#endif
    }

    uint8_t last_deg() const { return last_deg_; }

private:
    int pin_ = -1;
    int ch_ = 0;
    uint16_t min_us_ = 500, max_us_ = 2500;
    uint8_t last_deg_ = 255;
};
