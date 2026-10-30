#include "usb_audio.h"

#include <atomic>
#include <cmath>
#include <climits>

#include "hardware/sync.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "spsc_ring.h"

namespace {

constexpr uint32_t kBytesPerFrame = CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX;
static_assert(kBytesPerFrame == sizeof(StereoFrame), "StereoFrame must match the USB frame layout");

constexpr uint32_t kRingCapacityFrames = 256;
constexpr uint32_t kRingTargetFrames = 96;
constexpr uint32_t kReadChunkFrames = 96;
constexpr uint32_t kPrimeBytes = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2;

constexpr int16_t kVolumeMin = -50 * 256;
constexpr int16_t kVolumeMax = 0;
constexpr int16_t kVolumeRes = 1 * 256;
constexpr int16_t kVolumeSilence = static_cast<int16_t>(0x8000);

SpscRing<StereoFrame, kRingCapacityFrames> g_ring;

std::atomic<bool> g_streaming{false};
std::atomic<bool> g_active{false};
std::atomic<float> g_master_gain{1.0f};

bool g_primed = false;
uint32_t g_underruns = 0;

uint8_t g_mute = 0;
int16_t g_volume = kVolumeMax;

// Written from the USB ISR (core0) and read in usb_audio_take_stats() with interrupts disabled.
struct IsrStats {
    uint32_t fifo_min = UINT32_MAX;
    uint32_t fifo_max = 0;
    uint32_t rx_bytes = 0;
    uint32_t fifo_full_events = 0;
};
IsrStats g_isr;

void publish_master_gain() {
    float gain = 0.0f;
    if (!g_mute && g_volume != kVolumeSilence) {
        gain = powf(10.0f, (static_cast<float>(g_volume) / 256.0f) / 20.0f);
    }
    g_master_gain.store(gain, std::memory_order_relaxed);
}

void set_primed(bool primed) {
    g_primed = primed;
    g_active.store(primed, std::memory_order_release);
}

bool sampling_freq_set(const tusb_control_request_t* req, const uint8_t* buf) {
    if (req->bRequest != AUDIO10_CS_REQ_SET_CUR || req->wLength != 3) return false;
    const uint32_t freq = tu_unaligned_read32(buf) & 0x00FFFFFF;
    return freq == USB_AUDIO_SAMPLE_RATE_HZ;
}

bool sampling_freq_get(uint8_t rhport, const tusb_control_request_t* req) {
    if (req->bRequest != AUDIO10_CS_REQ_GET_CUR) return false;
    uint8_t freq[3] = {USB_AUDIO_SAMPLE_RATE_HZ & 0xFF, (USB_AUDIO_SAMPLE_RATE_HZ >> 8) & 0xFF, (USB_AUDIO_SAMPLE_RATE_HZ >> 16) & 0xFF};
    return tud_audio_buffer_and_schedule_control_xfer(rhport, req, freq, sizeof(freq));
}

}  // namespace

void usb_audio_init() {
    publish_master_gain();
}

void usb_audio_task() {
    if (!g_streaming.load(std::memory_order_acquire)) {
        if (g_primed) set_primed(false);
        if (tud_audio_available() > 0) tud_audio_clear_ep_out_ff();
        return;
    }

    const uint32_t fifo_frames = tud_audio_available() / kBytesPerFrame;
    const uint32_t fill = g_ring.size();

    if (!g_primed) {
        if (fill == 0 && tud_audio_available() >= kPrimeBytes) set_primed(true);
        return;
    }

    if (fill == 0 && fifo_frames == 0) {
        ++g_underruns;
        set_primed(false);
        return;
    }

    if (fill >= kRingTargetFrames || fifo_frames == 0) return;

    uint32_t frames = kRingTargetFrames - fill;
    if (frames > fifo_frames) frames = fifo_frames;
    if (frames > kReadChunkFrames) frames = kReadChunkFrames;

    StereoFrame buf[kReadChunkFrames];
    const uint32_t bytes = tud_audio_read(buf, static_cast<uint16_t>(frames * kBytesPerFrame));
    g_ring.write(buf, bytes / kBytesPerFrame);
}

UsbAudioStats usb_audio_take_stats() {
    UsbAudioStats s{};
    const uint32_t irq = save_and_disable_interrupts();
    s.fifo_min_bytes = (g_isr.fifo_min == UINT32_MAX) ? 0 : g_isr.fifo_min;
    s.fifo_max_bytes = g_isr.fifo_max;
    s.rx_frames = g_isr.rx_bytes / kBytesPerFrame;
    s.fifo_full_events = g_isr.fifo_full_events;
    g_isr = IsrStats{};
    restore_interrupts(irq);

    s.streaming = g_streaming.load(std::memory_order_relaxed);
    s.primed = g_primed;
    s.underruns = g_underruns;
    return s;
}

uint32_t usb_audio_pull(StereoFrame* dst, uint32_t frames) {
    return g_ring.read(dst, frames);
}

bool usb_audio_active() {
    return g_active.load(std::memory_order_acquire);
}

float usb_audio_master_gain() {
    return g_master_gain.load(std::memory_order_relaxed);
}

extern "C" {

void tud_umount_cb(void) {
    g_streaming.store(false, std::memory_order_release);
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
    (void)rhport;
    const uint8_t itf = tu_u16_low(p_request->wIndex);
    const uint8_t alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING) g_streaming.store(alt != 0, std::memory_order_release);
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
    (void)rhport;
    const uint8_t itf = tu_u16_low(p_request->wIndex);
    const uint8_t alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING && alt == 0) g_streaming.store(false, std::memory_order_release);
    return true;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff) {
    (void)rhport;
    if (TU_U16_HIGH(p_request->wValue) == AUDIO10_EP_CTRL_SAMPLING_FREQ) return sampling_freq_set(p_request, pBuff);
    return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
    if (TU_U16_HIGH(p_request->wValue) == AUDIO10_EP_CTRL_SAMPLING_FREQ) return sampling_freq_get(rhport, p_request);
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff) {
    (void)rhport;
    const uint8_t channel = TU_U16_LOW(p_request->wValue);
    const uint8_t ctrl_sel = TU_U16_HIGH(p_request->wValue);
    const uint8_t entity = TU_U16_HIGH(p_request->wIndex);

    if (entity != UAC1_ENTITY_FEATURE_UNIT || channel != 0 || p_request->bRequest != AUDIO10_CS_REQ_SET_CUR) return false;

    if (ctrl_sel == AUDIO10_FU_CTRL_MUTE) {
        TU_VERIFY(p_request->wLength == 1);
        g_mute = pBuff[0];
        publish_master_gain();
        return true;
    }
    if (ctrl_sel == AUDIO10_FU_CTRL_VOLUME) {
        TU_VERIFY(p_request->wLength == 2);
        int16_t volume = static_cast<int16_t>(tu_unaligned_read16(pBuff));
        if (volume != kVolumeSilence) {
            if (volume < kVolumeMin) volume = kVolumeMin;
            if (volume > kVolumeMax) volume = kVolumeMax;
        }
        g_volume = volume;
        publish_master_gain();
        return true;
    }
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
    const uint8_t channel = TU_U16_LOW(p_request->wValue);
    const uint8_t ctrl_sel = TU_U16_HIGH(p_request->wValue);
    const uint8_t entity = TU_U16_HIGH(p_request->wIndex);

    if (entity != UAC1_ENTITY_FEATURE_UNIT || channel != 0) return false;

    if (ctrl_sel == AUDIO10_FU_CTRL_MUTE && p_request->bRequest == AUDIO10_CS_REQ_GET_CUR) {
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &g_mute, sizeof(g_mute));
    }
    if (ctrl_sel == AUDIO10_FU_CTRL_VOLUME) {
        int16_t value;
        switch (p_request->bRequest) {
            case AUDIO10_CS_REQ_GET_CUR: value = g_volume; break;
            case AUDIO10_CS_REQ_GET_MIN: value = kVolumeMin; break;
            case AUDIO10_CS_REQ_GET_MAX: value = kVolumeMax; break;
            case AUDIO10_CS_REQ_GET_RES: value = kVolumeRes; break;
            default: return false;
        }
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &value, sizeof(value));
    }
    return false;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param) {
    (void)func_id;
    (void)alt_itf;
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = USB_AUDIO_SAMPLE_RATE_HZ;
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    const uint32_t level = tud_audio_available();
    if (level < g_isr.fifo_min) g_isr.fifo_min = level;
    if (level > g_isr.fifo_max) g_isr.fifo_max = level;
    if (level >= CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ) ++g_isr.fifo_full_events;
    g_isr.rx_bytes += n_bytes_received;
    return true;
}

}  // extern "C"
