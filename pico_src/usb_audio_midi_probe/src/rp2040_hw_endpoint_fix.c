/*
 * Backport of TinyUSB PR #2937 for pico-sdk 2.3.1 (TinyUSB 0.18).
 *
 * Starting a new RP2040 ISO transfer while the previous buffer is still
 * AVAILABLE panics ("ep XX was already available"). SET_INTERFACE during
 * streaming hits that path because dcd_edpt_iso_activate does not abort.
 */

#include "tusb.h"
#include "hardware/sync.h"
#include "rp2040_usb.h"

extern void __real_hw_endpoint_xfer_start(struct hw_endpoint *ep, uint8_t *buffer, uint16_t total_len);

static void abort_if_armed(struct hw_endpoint *ep) {
    if (ep->buffer_control == NULL) return;

    uint32_t const buf_ctrl = *ep->buffer_control;
    bool const avail = (buf_ctrl & USB_BUF_CTRL_AVAIL) || ((buf_ctrl >> 16) & USB_BUF_CTRL_AVAIL);
    if (!ep->active && !avail) return;

    uint8_t const dir = tu_edpt_dir(ep->ep_addr);
    uint8_t const epnum = tu_edpt_number(ep->ep_addr);
    uint32_t const abort_mask = TU_BIT((epnum << 1) | (dir ? 0 : 1));

    if (rp2040_chip_version() >= 2) {
        usb_hw_set->abort = abort_mask;
        while ((usb_hw->abort_done & abort_mask) != abort_mask) {
        }
    }

    uint32_t next_ctrl = USB_BUF_CTRL_SEL;
    if (ep->next_pid) next_ctrl |= USB_BUF_CTRL_DATA1_PID;
    _hw_endpoint_buffer_control_set_value32(ep, next_ctrl);
    hw_endpoint_reset_transfer(ep);
    usb_hw_clear->buf_status = abort_mask;

    if (rp2040_chip_version() >= 2) {
        usb_hw_clear->abort_done = abort_mask;
        usb_hw_clear->abort = abort_mask;
    }
}

void __wrap_hw_endpoint_xfer_start(struct hw_endpoint *ep, uint8_t *buffer, uint16_t total_len) {
    uint32_t const ints = save_and_disable_interrupts();
    abort_if_armed(ep);
    restore_interrupts(ints);
    __real_hw_endpoint_xfer_start(ep, buffer, total_len);
}
