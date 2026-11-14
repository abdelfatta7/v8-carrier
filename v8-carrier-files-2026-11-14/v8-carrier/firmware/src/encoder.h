// Quadrature encoder decoded in GPIO interrupts (x4: every edge of A and B).
// At 0.3 m/s on an 80 mm wheel with 1320 ticks/rev that is ~1600 edges/s per
// wheel, far below what the ISR path handles. The PCNT peripheral would do it
// without interrupts; swap this class for one if the tick rate ever matters.
#pragma once

#include <Arduino.h>

class QuadratureEncoder {
public:
    void begin(int pin_a, int pin_b, bool invert) {
        pin_a_ = pin_a;
        pin_b_ = pin_b;
        invert_ = invert;
        pinMode(pin_a_, INPUT_PULLUP);
        pinMode(pin_b_, INPUT_PULLUP);
        state_ = static_cast<uint8_t>((digitalRead(pin_a_) << 1) | digitalRead(pin_b_));
        attachInterruptArg(digitalPinToInterrupt(pin_a_), isr, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(pin_b_), isr, this, CHANGE);
    }

    int32_t read() const {
        noInterrupts();
        const int32_t c = count_;
        interrupts();
        return invert_ ? -c : c;
    }

private:
    static void IRAM_ATTR isr(void* arg) {
        QuadratureEncoder* self = static_cast<QuadratureEncoder*>(arg);
        const uint8_t now = static_cast<uint8_t>((digitalRead(self->pin_a_) << 1) | digitalRead(self->pin_b_));
        // Transition table indexed by (previous << 2 | current): +1, -1 or 0
        // (0 also for an impossible double transition = a missed edge).
        static const int8_t kTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        self->count_ += kTable[(self->state_ << 2) | now];
        self->state_ = now;
    }

    int pin_a_ = -1, pin_b_ = -1;
    bool invert_ = false;
    volatile uint8_t state_ = 0;
    volatile int32_t count_ = 0;
};
