#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Snapshot of the Joystick2 probe. Core1 updates it; core0 prints it.
// last_write / last_read / boot_write / boot_read are byte counts, or
// PICO_ERROR_GENERIC (-1, NACK or no device) / PICO_ERROR_TIMEOUT (-2).
// A read code of kReadNotStarted means the write did not complete.
struct JoystickProbeStatus {
    bool ok;
    uint8_t fw;
    int last_write;
    int last_read;
    uint32_t attempts;
    uint32_t last_try_ms;
    uint32_t ok_at_ms;
    uint32_t boot_fails;
    int boot_write;
    int boot_read;
    uint8_t sda;
    uint8_t scl;
    uint baud;
};

// M5Stack Unit Joystick2 (U024-V2) over Pico I2C.
// Protocol matches M5Unit-Joystick2 / m5_blue src/main.cpp.
class Joystick2 {
  public:
    static constexpr i2c_inst_t* kI2c = i2c0;
    static constexpr uint kSdaPin = 4;
    static constexpr uint kSclPin = 5;
    static constexpr uint8_t kAddr = 0x63;
    static constexpr uint8_t kButtonReg = 0x20;
    static constexpr uint8_t kOffsetAdc12Reg = 0x50;
    static constexpr uint8_t kFirmwareVersionReg = 0xFE;
    static constexpr uint kI2cHz = 100000;
    static constexpr int kReadNotStarted = -99;
    // One attempt. A retry loop on core0 blocks the USB audio drain.
    static constexpr uint8_t kI2cRetries = 1;
    // Whole-transfer budget. A 100 kHz button+axes read is about 1.1 ms; this still caps a stuck bus.
    static constexpr uint kI2cTimeoutUs = 1500;
    static constexpr uint8_t kBootProbes = 5;

    bool init() {
        bind_bus();
        ok_.store(false, std::memory_order_relaxed);
        uint32_t fails = 0;
        int fail_write = 0;
        int fail_read = kReadNotStarted;
        bool success = false;
        for (uint8_t attempt = 0; attempt < kBootProbes; attempt++) {
            if (probe_firmware()) {
                success = true;
                break;
            }
            fails++;
            fail_write = last_write_.load(std::memory_order_relaxed);
            fail_read = last_read_.load(std::memory_order_relaxed);
            sleep_ms(50);
        }
        boot_fails_.store(fails, std::memory_order_relaxed);
        boot_write_.store(fail_write, std::memory_order_relaxed);
        boot_read_.store(fail_read, std::memory_order_relaxed);

        const JoystickProbeStatus st = status();
        printf("Joystick2 boot %s fw=%u fails=%lu w=%d r=%d sda=%u scl=%u baud=%lu (SDA=GP%u SCL=GP%u)\n",
               success ? "ok" : "fail", st.fw, static_cast<unsigned long>(st.boot_fails), st.boot_write, st.boot_read, st.sda, st.scl,
               static_cast<unsigned long>(st.baud), kSdaPin, kSclPin);
        return success;
    }

    // Joystick2 is powered with the Pico. A cold USB insert can probe before its
    // firmware answers, and a one-shot failure used to disable the filter until reboot.
    bool reinit_and_probe() {
        bind_bus();
        return probe_firmware();
    }

    bool ok() const { return ok_.load(std::memory_order_relaxed); }

    JoystickProbeStatus status() const {
        JoystickProbeStatus st;
        st.ok = ok_.load(std::memory_order_relaxed);
        st.fw = fw_.load(std::memory_order_relaxed);
        st.last_write = last_write_.load(std::memory_order_relaxed);
        st.last_read = last_read_.load(std::memory_order_relaxed);
        st.attempts = attempts_.load(std::memory_order_relaxed);
        st.last_try_ms = last_try_ms_.load(std::memory_order_relaxed);
        st.ok_at_ms = ok_at_ms_.load(std::memory_order_relaxed);
        st.boot_fails = boot_fails_.load(std::memory_order_relaxed);
        st.boot_write = boot_write_.load(std::memory_order_relaxed);
        st.boot_read = boot_read_.load(std::memory_order_relaxed);
        st.sda = sda_.load(std::memory_order_relaxed);
        st.scl = scl_.load(std::memory_order_relaxed);
        st.baud = baud_.load(std::memory_order_relaxed);
        return st;
    }

    bool readButton(uint8_t* button_out) {
        for (uint8_t attempt = 0; attempt < kI2cRetries; attempt++) {
            uint8_t data = 0xFF;
            if (!readBytes(kButtonReg, &data, 1)) {
                continue;
            }
            // Documented values (same as m5_blue): 0 = pressed, 1 = released.
            if (data <= 1) {
                *button_out = data;
                return true;
            }
            return false;
        }
        return false;
    }

    bool readAxesOffset(int16_t* x_out, int16_t* y_out) {
        for (uint8_t attempt = 0; attempt < kI2cRetries; attempt++) {
            uint8_t data[4] = {0, 0, 0, 0};
            if (!readBytes(kOffsetAdc12Reg, data, sizeof(data))) {
                continue;
            }
            int16_t x = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
            int16_t y = (int16_t)(data[2] | ((uint16_t)data[3] << 8));

            // Match m5_blue orientation: negative Y is cable side / LED down.
            y = (int16_t)(-y);

            // 12-bit offsets are about ±4096; reject wild values from bad transfers.
            if (x >= -4500 && x <= 4500 && y >= -4500 && y <= 4500) {
                *x_out = x;
                *y_out = y;
                return true;
            }
        }
        return false;
    }

  private:
    std::atomic<bool> ok_{false};
    std::atomic<uint8_t> fw_{0};
    std::atomic<int> last_write_{0};
    std::atomic<int> last_read_{kReadNotStarted};
    std::atomic<uint32_t> attempts_{0};
    std::atomic<uint32_t> last_try_ms_{0};
    std::atomic<uint32_t> ok_at_ms_{0};
    std::atomic<uint32_t> boot_fails_{0};
    std::atomic<int> boot_write_{0};
    std::atomic<int> boot_read_{kReadNotStarted};
    std::atomic<uint8_t> sda_{1};
    std::atomic<uint8_t> scl_{1};
    std::atomic<uint> baud_{0};

    void bind_bus() {
        baud_.store(i2c_init(kI2c, kI2cHz), std::memory_order_relaxed);
        gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
        gpio_set_function(kSclPin, GPIO_FUNC_I2C);
        gpio_pull_up(kSdaPin);
        gpio_pull_up(kSclPin);
    }

    void note_levels() {
        sda_.store(static_cast<uint8_t>(gpio_get(kSdaPin)), std::memory_order_relaxed);
        scl_.store(static_cast<uint8_t>(gpio_get(kSclPin)), std::memory_order_relaxed);
    }

    bool probe_firmware() {
        uint8_t fw = 0;
        const uint8_t reg = kFirmwareVersionReg;
        const int wrote = i2c_write_timeout_us(kI2c, kAddr, &reg, 1, true, kI2cTimeoutUs);
        int got = kReadNotStarted;
        bool success = false;
        if (wrote == 1) {
            got = i2c_read_timeout_us(kI2c, kAddr, &fw, 1, false, kI2cTimeoutUs);
            success = got == 1;
        }
        note_levels();
        last_write_.store(wrote, std::memory_order_relaxed);
        last_read_.store(got, std::memory_order_relaxed);
        attempts_.fetch_add(1, std::memory_order_relaxed);
        last_try_ms_.store(to_ms_since_boot(get_absolute_time()), std::memory_order_relaxed);
        if (success) {
            fw_.store(fw, std::memory_order_relaxed);
            uint32_t expected = 0;
            ok_at_ms_.compare_exchange_strong(expected, last_try_ms_.load(std::memory_order_relaxed), std::memory_order_relaxed);
            ok_.store(true, std::memory_order_relaxed);
        }
        return success;
    }

    bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
        int wrote = i2c_write_timeout_us(kI2c, kAddr, &reg, 1, true, kI2cTimeoutUs);
        if (wrote != 1) {
            return false;
        }
        int got = i2c_read_timeout_us(kI2c, kAddr, buffer, length, false, kI2cTimeoutUs);
        return got == (int)length;
    }
};
