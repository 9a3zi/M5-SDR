/*
 * wspr.h — WSPR (Weak Signal Propagation Reporter) decoder.
 *
 * Input : demodulated USB audio, one sample at a time at 48 kHz, from dsp.c
 *         while in DSP_MODE_WSPR (wspr_feed). Decimated to 12 kHz here.
 * Output: list of decoded spots for the UI (wspr_get_messages).
 *
 * Independent implementation from the public protocol description (G4JNT,
 * "The WSPR Coding Process"): 4-FSK, 1.4648 baud, 162 symbols, rate-1/2 K=32
 * convolutional code with the sync vector in the LSB of each symbol.
 *
 * There is no clock on the device and WSPR transmissions are 111 s long, so the
 * decoder does not wait for slot boundaries: it keeps a rolling spectrogram of
 * the last two minutes, searches it for the sync pattern at every time offset
 * and decodes any signal as soon as its 162 symbols have been received.
 * Only message type 1 (callsign, 4-character locator, power) is shown; the
 * rarer extended types 2/3 are counted but not displayed.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WSPR_TEXT_MAX 32

typedef struct {
    uint32_t seq;
    int16_t snr;          /* approximate SNR in 2500 Hz, dB              */
    float dt;             /* time offset vs. the cycle grid, s           */
    float freq;           /* audio frequency of the lowest tone, Hz      */
    int8_t drift;         /* frequency drift over the message, Hz        */
    char text[WSPR_TEXT_MAX]; /* "K1ABC FN42 37"                         */
} wspr_msg_t;

typedef struct {
    bool grid_known;      /* 2-minute cycle phase known (after 1st decode) */
    float cycle_pos;      /* seconds into the 120 s cycle (if known)       */
    uint32_t total;       /* spots decoded since entering WSPR mode        */
    uint32_t other_types; /* decoded but not displayable (types 2/3)       */
    float buffered_s;     /* seconds of audio in the rolling spectrogram    */

    /* diagnostics: where does the chain stop? */
    uint32_t scans;       /* sync scans done                                */
    uint32_t cands;       /* sync candidates handed to the Fano decoder     */
    uint32_t fano_ok;     /* candidates where the Fano decoder converged    */
    uint32_t drops;       /* audio samples lost (decoder too slow)          */
    float best_m;         /* best sync correlation in the last scan (0..1; > 0.15 = candidate) */

    /* self-test (wspr_selftest): 0 = never run, 1 = running, 2 = passed, 3 = failed */
    int selftest_state;
    int selftest_pct;
    float selftest_speed; /* audio seconds processed per second of real time */
} wspr_status_t;

/* Allocate buffers and create the decoder task (once; the task is a no-op on the host). */
void wspr_start(void);

/* Forget everything (call when entering WSPR mode). */
void wspr_reset(void);

/* One 48 kHz audio sample (any scale; normalised internally). */
void wspr_feed(float y);

/* Copy up to `max` newest spots (oldest first). Returns the count. */
int wspr_get_messages(wspr_msg_t *out, int max, wspr_status_t *st);

/* Run a self-test: the decoder synthesises two WSPR transmissions ("K1ABC FN42 37", -20 dB,
 * with noise) and runs them through the real pipeline faster than real time. It shows
 * whether the decoder itself works on this hardware, independent of radio and propagation.
 * Live audio is ignored while it runs (tens of seconds). */
void wspr_selftest(void);

/* Consume pending audio, update the spectrogram, scan/decode when due. */
void wspr_process(void);

#ifdef __cplusplus
}
#endif
