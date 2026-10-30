#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "tusb.h"
#include "usb_descriptors.h"

#include <string.h>

static size_t board_usb_get_serial(uint16_t *desc_str1, size_t max_chars) {
    char buf[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(buf, sizeof(buf));
    size_t n = strlen(buf);
    if (n > max_chars) n = max_chars;
    for (size_t i = 0; i < n; i++) {
        desc_str1[i] = (uint16_t)buf[i];
    }
    return n;
}

#define _PID_MAP(itf, n) ((CFG_TUD_##itf) << (n))
#define USB_PID (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                 _PID_MAP(MIDI, 3) | _PID_MAP(AUDIO, 4) | _PID_MAP(VENDOR, 5))

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AUDIO,
    STRID_MIDI,
    STRID_CDC,
};

char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "m5_blue",
    "Pico USB DAC + MIDI + CDC",
    NULL,
    "Pico Headphones",
    "Pico MIDI",
    "Pico Serial",
};

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCafe,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

#define EPNUM_AUDIO_OUT 0x01
#define EPNUM_AUDIO_FB 0x01
#define EPNUM_MIDI_OUT 0x02
#define EPNUM_MIDI_IN 0x02
#define EPNUM_CDC_NOTIF 0x03
#define EPNUM_CDC_OUT 0x04
#define EPNUM_CDC_IN 0x04

#define CONFIG_TOTAL_LEN                                                                                 \
    (TUD_CONFIG_DESC_LEN + TUD_AUDIO10_HEADPHONE_STEREO_FB_DESC_LEN(1) + TUD_MIDI_DESC_IAD_LEN + \
     TUD_MIDI_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // UAC1 headphones, 48 kHz only (no Audio IAD).
    TUD_AUDIO10_HEADPHONE_STEREO_FB_DESCRIPTOR(
        ITF_NUM_AUDIO_CONTROL, STRID_AUDIO,
        CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX,
        EPNUM_AUDIO_OUT, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX, EPNUM_AUDIO_FB | 0x80,
        48000),

    TUD_MIDI_DESC_IAD(ITF_NUM_MIDI, 0x02, STRID_MIDI),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, STRID_MIDI, EPNUM_MIDI_OUT, (EPNUM_MIDI_IN | 0x80), 64),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, (EPNUM_CDC_NOTIF | 0x80), 8,
                       EPNUM_CDC_OUT, (EPNUM_CDC_IN | 0x80), 64),
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "Incorrect configuration descriptor size");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    switch (index) {
    case STRID_LANGID:
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
        break;

    case STRID_SERIAL:
        chr_count = board_usb_get_serial(_desc_str + 1, 32);
        break;

    default:
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }
        {
            const char *str = string_desc_arr[index];
            chr_count = strlen(str);
            size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
            if (chr_count > max_count) chr_count = max_count;
            for (size_t i = 0; i < chr_count; i++) {
                _desc_str[1 + i] = str[i];
            }
        }
        break;
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
