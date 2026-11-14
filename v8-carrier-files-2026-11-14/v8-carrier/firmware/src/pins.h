// Pin map for the ESP32-S3 DevKitC-1 (default decision 2026-11-10; change
// here only). Every GPIO below is free on the DevKitC-1 and is not a strapping
// pin (0, 3, 45, 46) nor used by the module's flash/PSRAM (26-37).
// ADC inputs must be on ADC1 (GPIO 1-10) because ADC2 is shared with Wi-Fi.
#pragma once

// ---- UART link to the Raspberry Pi 5 (3.3 V, cross TX/RX) ------------------
// Pi GPIO14 (TXD, pin 8)  -> ESP32 GPIO18 (RX)
// Pi GPIO15 (RXD, pin 10) <- ESP32 GPIO17 (TX)
// The USB-UART of the DevKit stays free for `pio device monitor` debug output.
#define PIN_LINK_TX 17
#define PIN_LINK_RX 18
#define LINK_BAUD 115200

// ---- Motor drivers (DRV8871, two inputs each, PWM on LEDC) -----------------
#define PIN_MOTOR_L_IN1 10
#define PIN_MOTOR_L_IN2 11
#define PIN_MOTOR_R_IN1 12
#define PIN_MOTOR_R_IN2 13
#define PIN_BRUSH_IN1 14
#define PIN_BRUSH_IN2 21
// Set to 1 if a wheel runs the wrong way; keeps the wiring untouched.
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 0

// ---- Quadrature encoders (JGB37-520 hall encoders, 3.3 V logic) -----------
#define PIN_ENC_L_A 6
#define PIN_ENC_L_B 7
#define PIN_ENC_R_A 15
#define PIN_ENC_R_B 16
#define ENC_L_INVERT 0
#define ENC_R_INVERT 1  // the right motor is mounted mirrored
// Ticks per wheel revolution: 11 pulses/rev on the motor x 4 (quadrature
// edges) x gearbox ratio. JGB37-520 comes in many ratios; 1:30 is the
// default assumption here. Measure it: one wheel turn by hand must give this
// number on the status message.
#define ENCODER_TICKS_PER_REV 1320

// ---- I2C bus (BNO085 IMU at 0x4A, INA226 at 0x40) --------------------------
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define INA226_ADDR 0x40
#define INA226_SHUNT_OHMS 0.010f  // 10 mOhm shunt board (8 A full scale)
#define BNO085_ADDR 0x4A
#define MPU6050_ADDR 0x68

// ---- Bumper microswitches (lever, front corners; switch to GND, pull-up) --
#define PIN_BUMPER_L 39
#define PIN_BUMPER_R 40

// ---- Cliff sensors (Sharp GP2Y0A51SK0F analog out, ADC1) -------------------
// Order matches StatusMsg::cliff_mv: nose-left, nose-right, wheel-left, wheel-right.
#define PIN_CLIFF_0 1
#define PIN_CLIFF_1 2
#define PIN_CLIFF_2 4
#define PIN_CLIFF_3 5

// ---- INMP441 I2S microphone -------------------------------------------------
#define PIN_I2S_BCLK 41
#define PIN_I2S_WS 42
#define PIN_I2S_DIN 47
#define MIC_SAMPLE_RATE 16000
#define MIC_WINDOW_SAMPLES 1024  // 64 ms per RMS value

// ---- Hobby servo for the Dyson's power button (LEDC, 50 Hz) ----------------
#define PIN_SERVO 38
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// ---- Status LED (the DevKitC-1 has an addressable LED on 48; a plain LED
// on a free pin is simpler) --------------------------------------------------
#define PIN_LED 48
