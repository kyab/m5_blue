#include "usb_audio.h"

#include "tusb.h"
#include "usb_descriptors.h"

#include <cstring>

namespace {

AudioSharedState g_shared;

constexpr uint32_t kFrameBytes =
    CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX;
constexpr uint32_t kSampleRate = CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE;

// Wait for TinyUSB FIFO to reach ~half before first drain (FIFO_COUNT target).
constexpr uint16_t kFifoStartBytes = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2;
// Keep the handoff ring near 2 ms so FIFO_COUNT still sees backlog.
constexpr size_t kRingTargetFrames = (kSampleRate * 2) / 1000;

uint8_t mute_ctrl[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {0};
// Stored as integer dB (UAC1 sample convention); wire format is 1/256 dB.
int16_t volume_db[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {0};

bool drain_armed = false;
uint32_t current_sample_rate = kSampleRate;

alignas(4) int16_t read_scratch[49 * 2];

void publish_volume() {
    g_shared.host_mute.store(mute_ctrl[0] != 0, std::memory_order_relaxed);
    g_shared.host_volume_db.store(volume_db[0], std::memory_order_relaxed);
}

void note_fifo_level(uint16_t avail) {
    uint32_t mn = g_shared.fifo_min.load(std::memory_order_relaxed);
    while (avail < mn && !g_shared.fifo_min.compare_exchange_weak(mn, avail, std::memory_order_relaxed)) {
    }
    uint32_t mx = g_shared.fifo_max.load(std::memory_order_relaxed);
    while (avail > mx && !g_shared.fifo_max.compare_exchange_weak(mx, avail, std::memory_order_relaxed)) {
    }
}

bool audio10_set_req_ep(tusb_control_request_t const *p_request, uint8_t *pBuff) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ && p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        TU_VERIFY(p_request->wLength == 3);
        current_sample_rate = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
        return true;
    }
    return false;
}

bool audio10_get_req_ep(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ && p_request->bRequest == AUDIO10_CS_REQ_GET_CUR) {
        uint8_t freq[3] = {
            (uint8_t)(current_sample_rate & 0xFF),
            (uint8_t)((current_sample_rate >> 8) & 0xFF),
            (uint8_t)((current_sample_rate >> 16) & 0xFF),
        };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
    }
    return false;
}

bool audio10_set_req_entity(tusb_control_request_t const *p_request, uint8_t *pBuff) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;
    if (channelNum > CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) return false;

    if (ctrlSel == AUDIO10_FU_CTRL_MUTE && p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        TU_VERIFY(p_request->wLength == 1);
        mute_ctrl[channelNum] = pBuff[0];
        publish_volume();
        return true;
    }
    if (ctrlSel == AUDIO10_FU_CTRL_VOLUME && p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        TU_VERIFY(p_request->wLength == 2);
        int16_t const raw = (int16_t)tu_unaligned_read16(pBuff);
        int16_t db = (int16_t)(raw / 256);
        if (db > 0) db = 0;
        if (db < -50) db = -50;
        volume_db[channelNum] = db;
        publish_volume();
        return true;
    }
    return false;
}

bool audio10_get_req_entity(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;
    if (channelNum > CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) return false;

    if (ctrlSel == AUDIO10_FU_CTRL_MUTE) {
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute_ctrl[channelNum], 1);
    }
    if (ctrlSel == AUDIO10_FU_CTRL_VOLUME) {
        switch (p_request->bRequest) {
        case AUDIO10_CS_REQ_GET_CUR: {
            int16_t vol = (int16_t)(volume_db[channelNum] * 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &vol, sizeof(vol));
        }
        case AUDIO10_CS_REQ_GET_MIN: {
            int16_t min = (int16_t)(-50 * 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
        }
        case AUDIO10_CS_REQ_GET_MAX: {
            int16_t max = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
        }
        case AUDIO10_CS_REQ_GET_RES: {
            int16_t res = 256; // 1 dB
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
        }
        default:
            return false;
        }
    }
    return false;
}

} // namespace

AudioSharedState &audio_shared() { return g_shared; }

void usb_audio_init() {
    g_shared.ring.reset();
    publish_volume();
}

void usb_audio_poll() {
    if (!g_shared.streaming.load(std::memory_order_relaxed)) {
        drain_armed = false;
        return;
    }

    uint16_t const avail = tud_audio_available();
    note_fifo_level(avail);

    if (!drain_armed) {
        if (avail < kFifoStartBytes) return;
        drain_armed = true;
    }

    size_t const level = g_shared.ring.size();
    if (level >= kRingTargetFrames) return;

    size_t want_frames = kRingTargetFrames - level;
    size_t const free = g_shared.ring.free_space();
    if (want_frames > free) want_frames = free;
    if (want_frames == 0) return;

    size_t const avail_frames = avail / kFrameBytes;
    if (avail_frames == 0) return;
    if (want_frames > avail_frames) want_frames = avail_frames;
    if (want_frames > 49) want_frames = 49;

    uint16_t const got = tud_audio_read(read_scratch, (uint16_t)(want_frames * kFrameBytes));
    size_t const frames = got / kFrameBytes;
    if (frames == 0) return;

    size_t const pushed = g_shared.ring.push(read_scratch, frames);
    if (pushed < frames) {
        g_shared.overflow_count.fetch_add(1, std::memory_order_relaxed);
    }
}

//--------------------------------------------------------------------+
// TinyUSB callbacks (core0 / USB context only — no printf)
//--------------------------------------------------------------------+

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex);
    uint8_t const alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING) {
        g_shared.streaming.store(alt != 0, std::memory_order_relaxed);
        if (alt == 0) {
            drain_armed = false;
            g_shared.ring.reset();
        } else {
            drain_armed = false;
            g_shared.fifo_min.store(UINT32_MAX, std::memory_order_relaxed);
            g_shared.fifo_max.store(0, std::memory_order_relaxed);
        }
    }
    return true;
}

extern "C" bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex);
    uint8_t const alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING && alt == 0) {
        g_shared.streaming.store(false, std::memory_order_relaxed);
        drain_armed = false;
        g_shared.ring.reset();
    }
    return true;
}

extern "C" bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void)rhport;
    if (tud_audio_version() == 1) return audio10_set_req_ep(p_request, pBuff);
    return false;
}

extern "C" bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    if (tud_audio_version() == 1) return audio10_get_req_ep(rhport, p_request);
    return false;
}

extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void)rhport;
    if (tud_audio_version() == 1) return audio10_set_req_entity(p_request, buf);
    return false;
}

extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    if (tud_audio_version() == 1) return audio10_get_req_entity(rhport, p_request);
    return false;
}

extern "C" void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
    (void)func_id;
    (void)alt_itf;
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = current_sample_rate;
}
