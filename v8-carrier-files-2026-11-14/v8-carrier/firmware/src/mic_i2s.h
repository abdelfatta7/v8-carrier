// INMP441 on I2S, read in a FreeRTOS task that keeps an RMS level and the
// "vacuum running" flag up to date for the main loop. Uses the legacy
// driver/i2s.h API which exists in both Arduino-ESP32 2.x (IDF 4.4) and 3.x
// (IDF 5, deprecated but present). Hardware layer (Arduino).
#pragma once

#include <Arduino.h>
#include <driver/i2s.h>

#include "miclevel.h"

class MicI2s {
public:
    bool begin(int bclk, int ws, int din, uint32_t sample_rate, size_t window_samples,
               const v8c::MicDetectorConfig& det) {
        rms_ = v8c::RmsWindow(window_samples);
        detector_ = v8c::RunningDetector(det);

        i2s_config_t cfg = {};
        cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
        cfg.sample_rate = sample_rate;
        cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;  // INMP441 sends 24 bits left-justified in 32
        cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;    // L/R pin tied to GND
        cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
        cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
        cfg.dma_buf_count = 4;
        cfg.dma_buf_len = 256;
        cfg.use_apll = false;
        if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) return false;

        i2s_pin_config_t pins = {};
        pins.mck_io_num = I2S_PIN_NO_CHANGE;
        pins.bck_io_num = bclk;
        pins.ws_io_num = ws;
        pins.data_out_num = I2S_PIN_NO_CHANGE;
        pins.data_in_num = din;
        if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;

        running_task_ = xTaskCreatePinnedToCore(task_entry, "mic", 4096, this, 1, nullptr, 0) == pdPASS;
        return running_task_;
    }

    uint16_t level() const { return level_; }
    bool running() const { return running_; }
    bool ok() const { return running_task_; }

private:
    static void task_entry(void* arg) { static_cast<MicI2s*>(arg)->task(); }

    void task() {
        static int32_t raw[256];
        static int16_t samples[256];
        for (;;) {
            size_t bytes = 0;
            if (i2s_read(I2S_NUM_0, raw, sizeof raw, &bytes, portMAX_DELAY) != ESP_OK) continue;
            const size_t n = bytes / sizeof(int32_t);
            // 24-bit sample in the top bits of 32: shift to a 16-bit scale.
            for (size_t i = 0; i < n; ++i) samples[i] = static_cast<int16_t>(raw[i] >> 16);
            const size_t windows = rms_.add_block(samples, n);
            if (windows) {
                level_ = rms_.level();
                running_ = detector_.update(millis(), level_);
            }
        }
    }

    v8c::RmsWindow rms_{1024};
    v8c::RunningDetector detector_;
    volatile uint16_t level_ = 0;
    volatile bool running_ = false;
    bool running_task_ = false;
};
