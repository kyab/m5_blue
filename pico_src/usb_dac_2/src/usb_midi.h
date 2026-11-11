#pragma once

void usb_midi_init();
// Drain host→device MIDI (call every core0 loop).
void usb_midi_task();
// Dual Button GPIO + synth trigger / note edges. Caller paces (~2 ms with Joystick2).
void usb_midi_poll_buttons();
