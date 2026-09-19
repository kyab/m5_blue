#pragma once

#include <cstdint>
#include <cstdio>

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
    static constexpr uint8_t kI2cRetries = 3;

    bool init() {
        i2c_init(kI2c, kI2cHz);
        gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
        gpio_set_function(kSclPin, GPIO_FUNC_I2C);
        gpio_pull_up(kSdaPin);
        gpio_pull_up(kSclPin);

        uint8_t fw = 0;
        for (uint8_t attempt = 0; attempt < 5; attempt++) {
            if (readBytes(kFirmwareVersionReg, &fw, 1)) {
                printf("Joystick2 OK addr=0x%02X fw=%u (SDA=GP%u SCL=GP%u)\n",
                       kAddr, (unsigned)fw, kSdaPin, kSclPin);
                ok_ = true;
                return true;
            }
            sleep_ms(50);
        }

        printf("Joystick2 not found on I2C0 (SDA=GP%u SCL=GP%u)\n", kSdaPin, kSclPin);
        ok_ = false;
        return false;
    }

    bool ok() const { return ok_; }

    bool readButton(uint8_t* button_out) {
        for (uint8_t attempt = 0; attempt < kI2cRetries; attempt++) {
            uint8_t data = 0xFF;
            if (!readBytes(kButtonReg, &data, 1)) {
                continue;
            }
            // Raw I2C is inverted vs physical feel on this unit; report 0=released, 1=pressed.
            if (data == 1) {
                *button_out = 0;
                return true;
            }
            if (data == 0) {
                *button_out = 1;
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
    bool ok_ = false;

    bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
        int wrote = i2c_write_blocking(kI2c, kAddr, &reg, 1, true);
        if (wrote != 1) {
            return false;
        }
        int got = i2c_read_blocking(kI2c, kAddr, buffer, length, false);
        return got == (int)length;
    }
};
