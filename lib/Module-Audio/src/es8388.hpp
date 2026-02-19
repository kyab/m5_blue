/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __ES8388_HPP
#define __ES8388_HPP

#include <Arduino.h>
#include <Wire.h>

// #define ES8388_DEBUG Serial  // This macro definition can be annotated without sending and receiving data prints
//         Define the serial port you want to use, e.g., Serial1 or Serial2
#if defined ES8388_DEBUG
#define serialPrint(...)   ES8388_DEBUG.print(__VA_ARGS__)
#define serialPrintln(...) ES8388_DEBUG.println(__VA_ARGS__)
#define serialPrintf(...)  ES8388_DEBUG.printf(__VA_ARGS__)
#define serialFlush()      ES8388_DEBUG.flush()
#else
#endif

/**
 * @file ES8388_registers.h
 * @brief Definitions for ES8388 audio codec registers
 */

/**
 *  @brief I2C device address for ES8388
 */
#define ES8388_ADDR (0x10)

/* ES8388 control registers */
#define ES8388_CONTROL1 (0x00)
#define ES8388_CONTROL2 (0x01)
#define ES8388_CHIPPOWER (0x02)
#define ES8388_ADCPOWER (0x03)
#define ES8388_DACPOWER (0x04)
#define ES8388_CHIPLOPOW1 (0x05)
#define ES8388_CHIPLOPOW2 (0x06)
#define ES8388_ANAVOLMANAG (0x07)
#define ES8388_MASTERMODE (0x08)
#define ES8388_ADCCONTROL1 (0x09)
#define ES8388_ADCCONTROL2 (0x0a)
#define ES8388_ADCCONTROL3 (0x0b)
#define ES8388_ADCCONTROL4 (0x0c)
#define ES8388_ADCCONTROL5 (0x0d)
#define ES8388_ADCCONTROL6 (0x0e)
#define ES8388_ADCCONTROL7 (0x0f)
#define ES8388_ADCCONTROL8 (0x10)
#define ES8388_ADCCONTROL9 (0x11)
#define ES8388_ADCCONTROL10 (0x12)
#define ES8388_ADCCONTROL11 (0x13)
#define ES8388_ADCCONTROL12 (0x14)
#define ES8388_ADCCONTROL13 (0x15)
#define ES8388_ADCCONTROL14 (0x16)
#define ES8388_DACCONTROL1 (0x17)
#define ES8388_DACCONTROL2 (0x18)
#define ES8388_DACCONTROL3 (0x19)
#define ES8388_DACCONTROL4 (0x1a)
#define ES8388_DACCONTROL5 (0x1b)
#define ES8388_DACCONTROL6 (0x1c)
#define ES8388_DACCONTROL7 (0x1d)
#define ES8388_DACCONTROL8 (0x1e)
#define ES8388_DACCONTROL9 (0x1f)
#define ES8388_DACCONTROL10 (0x20)
#define ES8388_DACCONTROL11 (0x21)
#define ES8388_DACCONTROL12 (0x22)
#define ES8388_DACCONTROL13 (0x23)
#define ES8388_DACCONTROL14 (0x24)
#define ES8388_DACCONTROL15 (0x25)
#define ES8388_DACCONTROL16 (0x26)
#define ES8388_DACCONTROL17 (0x27)
#define ES8388_DACCONTROL18 (0x28)
#define ES8388_DACCONTROL19 (0x29)
#define ES8388_DACCONTROL20 (0x2a)
#define ES8388_DACCONTROL21 (0x2b)
#define ES8388_DACCONTROL22 (0x2c)
#define ES8388_DACCONTROL23 (0x2d)
#define ES8388_DACCONTROL24 (0x2e)
#define ES8388_DACCONTROL25 (0x2f)
#define ES8388_DACCONTROL26 (0x30)
#define ES8388_DACCONTROL27 (0x31)
#define ES8388_DACCONTROL28 (0x32)
#define ES8388_DACCONTROL29 (0x33)
#define ES8388_DACCONTROL30 (0x34)

typedef enum { MIXLIN1, MIXLIN2, MIXRES, MIXADC } es_mixsel_t;
typedef enum { ES_MODULE_MIN = -1, ES_MODULE_ADC = 0x01, ES_MODULE_DAC = 0x02, ES_MODULE_ADC_DAC = 0x03, ES_MODULE_LINE = 0x04, ES_MODULE_MAX } es_module_t;
typedef enum { BIT_LENGTH_MIN = -1, BIT_LENGTH_16BITS = 0x03, BIT_LENGTH_18BITS = 0x02, BIT_LENGTH_20BITS = 0x01, BIT_LENGTH_24BITS = 0x00, BIT_LENGTH_32BITS = 0x04, BIT_LENGTH_MAX } es_bits_length_t;
typedef enum { ADC_INPUT_LINPUT1_RINPUT1 = 0x00, ADC_INPUT_LINPUT2_RINPUT2 = 0x10, ADC_INPUT_DIFFERENCE1 = 0xf0 } es_adc_input_t;
typedef enum { DAC_OUTPUT_OUT1 = 0x30, DAC_OUTPUT_OUT2 = 0x0C, DAC_OUTPUT_ALL = 0x3c } es_dac_output_t;
typedef enum { MIC_GAIN_0DB = 0, MIC_GAIN_3DB, MIC_GAIN_6DB, MIC_GAIN_9DB, MIC_GAIN_12DB, MIC_GAIN_15DB, MIC_GAIN_18DB, MIC_GAIN_21DB, MIC_GAIN_24DB } es_mic_gain_t;
typedef enum { SAMPLE_RATE_8K = 0, SAMPLE_RATE_11K, SAMPLE_RATE_16K, SAMPLE_RATE_24K, SAMPLE_RATE_32K, SAMPLE_RATE_44K, SAMPLE_RATE_48K } es_sample_rate_t;

class ES8388 {
public:
    ES8388(TwoWire* wire, uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 400000L);
    uint8_t* readAllReg();
    bool applyResetSafeDacState();
    bool init();
    bool setMicGain(es_mic_gain_t gain);
    uint8_t getMicGain();
    bool setADCInput(es_adc_input_t input);
    bool setADCVolume(uint8_t volume);
    bool setDACOutput(es_dac_output_t output);
    bool setDACVolume(uint8_t volume);
    bool setDACmute(bool mute);
    bool setMixSourceSelect(es_mixsel_t lmixsel, es_mixsel_t rmixsel);
    bool setBitsSample(es_module_t mode, es_bits_length_t bits_len);
    bool setSampleRate(es_sample_rate_t rate);

private:
    TwoWire* _wire;
    uint8_t _scl;
    uint8_t _sda;
    uint32_t _speed;
    bool writeBytes(uint8_t reg, uint8_t data);
    bool readBytes(uint8_t reg, uint8_t& data);
};

#endif
