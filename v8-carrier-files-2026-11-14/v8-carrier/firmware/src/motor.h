// DRV8871 H-bridge on two LEDC PWM channels. Hardware layer (Arduino).
// Works with Arduino-ESP32 core 2.x (ledcSetup/ledcAttachPin) and 3.x (ledcAttach).
#pragma once

#include <Arduino.h>

class Drv8871 {
public:
    static constexpr uint32_t kPwmHz = 20000;  // above hearing
    static constexpr uint8_t kPwmBits = 10;
    static constexpr uint32_t kPwmMax = (1u << kPwmBits) - 1;

    // channel: LEDC channel index for core 2.x (two consecutive channels used).
    void begin(int in1, int in2, bool invert, int channel) {
        in1_ = in1;
        in2_ = in2;
        invert_ = invert;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        (void)channel;
        ledcAttach(in1_, kPwmHz, kPwmBits);
        ledcAttach(in2_, kPwmHz, kPwmBits);
#else
        ch1_ = channel;
        ch2_ = channel + 1;
        ledcSetup(ch1_, kPwmHz, kPwmBits);
        ledcSetup(ch2_, kPwmHz, kPwmBits);
        ledcAttachPin(in1_, ch1_);
        ledcAttachPin(in2_, ch2_);
#endif
        coast();
    }

    // duty in [-1, 1]; sign is direction. Slow-decay drive: the "on" side is
    // held high and the other side carries the inverted PWM, which gives
    // better low-speed torque than coasting between pulses.
    void set(float duty) {
        if (duty > 1.0f) duty = 1.0f;
        if (duty < -1.0f) duty = -1.0f;
        if (invert_) duty = -duty;
        const uint32_t d = static_cast<uint32_t>(fabsf(duty) * kPwmMax);
        if (d == 0) {
            brake();
        } else if (duty > 0) {
            write(in1_, ch1_, kPwmMax);
            write(in2_, ch2_, kPwmMax - d);
        } else {
            write(in1_, ch1_, kPwmMax - d);
            write(in2_, ch2_, kPwmMax);
        }
    }

    void brake() {  // both high: short the winding
        write(in1_, ch1_, kPwmMax);
        write(in2_, ch2_, kPwmMax);
    }
    void coast() {  // both low: free-wheel
        write(in1_, ch1_, 0);
        write(in2_, ch2_, 0);
    }

private:
    void write(int pin, int ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        (void)ch;
        ledcWrite(pin, duty);
#else
        (void)pin;
        ledcWrite(ch, duty);
#endif
    }
    int in1_ = -1, in2_ = -1;
    int ch1_ = 0, ch2_ = 1;
    bool invert_ = false;
};
