#pragma once

#include <atomic>
#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

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
    // One attempt. A retry loop on core0 blocks the USB audio drain.
    static constexpr uint8_t kI2cRetries = 1;
    // Whole-transfer budget. A 100 kHz button+axes read is about 1.1 ms; this still caps a stuck bus.
    static constexpr uint kI2cTimeoutUs = 1500;

    // Bus setup only. A cold USB insert does not ACK for the first few hundred
    // milliseconds, and probing here used to block core0 and then give up.
    void init() {
        i2c_init(kI2c, kI2cHz);
        gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
        gpio_set_function(kSclPin, GPIO_FUNC_I2C);
        gpio_pull_up(kSdaPin);
        gpio_pull_up(kSclPin);
        ok_.store(false, std::memory_order_relaxed);
    }

    bool probe() {
        uint8_t fw = 0;
        const uint8_t reg = kFirmwareVersionReg;
        const int wrote = i2c_write_timeout_us(kI2c, kAddr, &reg, 1, true, kI2cTimeoutUs);
        if (wrote != 1) {
            return false;
        }
        const int got = i2c_read_timeout_us(kI2c, kAddr, &fw, 1, false, kI2cTimeoutUs);
        if (got != 1) {
            return false;
        }
        ok_.store(true, std::memory_order_relaxed);
        return true;
    }

    bool ok() const { return ok_.load(std::memory_order_relaxed); }

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

    bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
        int wrote = i2c_write_timeout_us(kI2c, kAddr, &reg, 1, true, kI2cTimeoutUs);
        if (wrote != 1) {
            return false;
        }
        int got = i2c_read_timeout_us(kI2c, kAddr, buffer, length, false, kI2cTimeoutUs);
        return got == (int)length;
    }
};
