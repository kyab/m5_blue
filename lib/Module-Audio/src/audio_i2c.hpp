/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __AUDIO_I2C_HPP
#define __AUDIO_I2C_HPP

#include "Arduino.h"
#include "Wire.h"

#define I2C_ADDR (0x33)
#define I2C_ADDR_MIN (0x08)
#define I2C_ADDR_MAX (0x77)
#define MICROPHONE_STATUS (0x00)
#define HEADPHONE_MODE (0x10)
#define HEADPHONE_INSERT_STATUS (0x20)
#define RGB_LED_BRIGHTNESS (0x30)
#define RGB_LED (0x40)
#define FLASH_WRITE (0xF0)
#define FIRMWARE_VERSION (0xFE)
#define I2C_ADDRESS (0xFF)

typedef enum { AUDIO_MIC_CLOSE = 0, AUDIO_MIC_OPEN } audio_mic_t;
typedef enum { AUDIO_HPMODE_NATIONAL = 0, AUDIO_HPMODE_AMERICAN } audio_hpmode_t;

class AudioI2c {
public:
    bool begin(TwoWire* wire, uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 400000L, uint8_t addr = I2C_ADDR);
    void setMICStatus(audio_mic_t status);
    void setHPMode(audio_hpmode_t mode);
    void setRGBBrightness(uint8_t brightness);
    void setRGBLED(uint8_t num, uint32_t color);
    void setFlashWriteBack();
    uint8_t setI2CAddress(uint8_t newAddr);
    audio_mic_t getMICStatus();
    audio_hpmode_t getHPMode();
    uint8_t getHPInsertStatus();
    uint8_t getRGBBrightness();
    uint32_t getRGBLED(uint8_t num);
    uint8_t getFirmwareVersion();
    uint8_t getI2CAddress();

private:
    TwoWire* _wire = &Wire;
    uint8_t _addr;
    uint8_t _scl;
    uint8_t _sda;
    uint32_t _speed;
    void writeBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length);
    void readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length);
};
#endif
