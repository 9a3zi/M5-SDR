/*
 * audio.h — Tab5 speaker output for demodulated audio.
 *
 * Decouples the (real-time, must-not-block) demod path from the (blocking) I2S
 * codec writes via a StreamBuffer drained by a dedicated task.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_SAMPLE_RATE 48000

/* Volume applied at boot, before the user touches the UI. Kept low so an
 * unattended power-on isn't at full tilt; the UI seeds its own slider from it. */
#define AUDIO_VOL_DEFAULT 10

/* Bring up the ES8388 codec and start the output task. */
void audio_init(void);

/* Queue n mono samples for playback (duplicated to L/R). Non-blocking: if the
 * buffer is full the excess is dropped rather than stalling the caller. */
void audio_play(const int16_t *mono, int n);

/* Output volume, 0..100. */
void audio_set_volume(int vol);

/* Nonblocking UI touch confirmation tone, mixed by audio output task. */
void audio_touch_beep(void);

/* TX monitor: mute RTL-SDR audio while TX is active; tone = 1500 + symbol*6.25 Hz. */
void audio_ft8_tx_monitor_set_ft4(bool ft4);
void audio_ft8_tx_monitor_start(void);
void audio_ft8_tx_monitor_symbol(uint8_t symbol);
void audio_ft8_tx_monitor_stop(void);
/* TX monitor digital amplitude, 0..100 percent, independent of RX volume. */
void audio_ft8_tx_monitor_set_volume(int percent);
int audio_ft8_tx_monitor_get_volume(void);

#ifdef __cplusplus
}
#endif
