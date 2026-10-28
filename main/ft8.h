/*
 * ft8.h — FT8 decoder glue (protocol code: ft8lib/, MIT, by K. Goba).
 *
 * Input : demodulated USB audio, one sample at a time at 48 kHz, from dsp.c
 *         while in DSP_MODE_FT8 (ft8_feed). Decimated to 12 kHz here.
 * Output: list of decoded messages for the UI (ft8_get_messages).
 *
 * There is no wall clock in this device. FT8 transmissions are aligned to
 * 15-second UTC slots, so the decoder finds the slot phase itself: while
 * "scanning" it tries a decode on the latest 15 s of audio every ~2.5 s; the
 * first successful decode reveals where the slot boundaries are, and from then
 * on it decodes once per slot ("locked"). The slot phase is kept by counting
 * audio samples (stable to a few ppm, driven by the dongle's TCXO).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FT8_TEXT_MAX 36

typedef struct {
    uint32_t seq;        /* increments per decoded message                 */
    uint16_t slot;       /* slot counter (since lock), wraps               */
    int16_t snr;         /* approximate SNR in 2500 Hz, dB                 */
    float dt;            /* time offset vs. the slot grid, s (nominal 0)   */
    int16_t freq;        /* audio frequency of tone 0, Hz                  */
    char text[FT8_TEXT_MAX];
} ft8_msg_t;

typedef struct {
    bool locked;         /* slot phase known                               */
    float slot_pos;      /* seconds into the current slot (valid if locked) */
    uint32_t slots;      /* slots decoded since lock                       */
    uint32_t total;      /* messages decoded since entering FT8 mode       */
    int last_count;      /* messages decoded in the last attempt           */
} ft8_status_t;

/* Create the decoder task (call once at start-up; no-op on the host). */
void ft8_start(void);

/* Forget everything (call when entering FT8 mode). Safe from any task. */
void ft8_reset(void);

/* One 48 kHz audio sample (any scale; normalised internally). Called by the
 * DSP task; cheap. */
void ft8_feed(float y);

/* Copy up to `max` newest messages (oldest first). Returns the count. */
int ft8_get_messages(ft8_msg_t *out, int max, ft8_status_t *st);

/* Do all pending work (consume audio, build waterfall, decode when due).
 * Called by the task; exposed for host tests. */
void ft8_process(void);

#ifdef __cplusplus
}
#endif
