#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"
#include "usb_descriptors.h"

#define USB_VID 0xCafe
#define USB_PID 0x4A02

#define EPNUM_AUDIO_OUT 0x01
#define EPNUM_AUDIO_FB 0x01
#define EPNUM_MIDI_OUT 0x02
#define EPNUM_MIDI_IN 0x02
#define EPNUM_CDC_NOTIF 0x03
#define EPNUM_CDC_OUT 0x04
#define EPNUM_CDC_IN 0x04

#define MIDI_EP_SIZE 64
#define CDC_EP_SIZE 64
#define CDC_NOTIF_EP_SIZE 8

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_AUDIO10_HEADPHONES_STEREO_FB_DESC_LEN + TUD_MIDI_DESC_IAD_LEN + TUD_MIDI_DESC_LEN + TUD_CDC_DESC_LEN)

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AUDIO,
    STRID_MIDI,
    STRID_CDC,
};

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_AUDIO10_HEADPHONES_STEREO_FB_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, STRID_AUDIO, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX, EPNUM_AUDIO_OUT, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX, EPNUM_AUDIO_FB | 0x80, USB_AUDIO_SAMPLE_RATE_HZ),
    TUD_MIDI_DESC_IAD(ITF_NUM_MIDI_CONTROL, 0x02, STRID_MIDI),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI_CONTROL, STRID_MIDI, EPNUM_MIDI_OUT, EPNUM_MIDI_IN | 0x80, MIDI_EP_SIZE),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF | 0x80, CDC_NOTIF_EP_SIZE, EPNUM_CDC_OUT, EPNUM_CDC_IN | 0x80, CDC_EP_SIZE),
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "Incorrect configuration descriptor size");

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static char const* const string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "m5_blue",
    "Pico USB DAC",
    NULL,
    "Pico USB DAC",
    "Pico USB DAC MIDI",
    "Pico USB DAC Serial",
};

static uint16_t desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    if (index == STRID_LANGID) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == STRID_SERIAL) {
        char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
        pico_get_unique_board_id_string(serial, sizeof(serial));
        chr_count = strlen(serial);
        if (chr_count > 32) chr_count = 32;
        for (size_t i = 0; i < chr_count; i++) desc_str[1 + i] = (uint16_t)serial[i];
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        char const* str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 32) chr_count = 32;
        for (size_t i = 0; i < chr_count; i++) desc_str[1 + i] = (uint16_t)str[i];
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
