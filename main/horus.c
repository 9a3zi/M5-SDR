#include "horus.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

/* RTL-SDR phase discriminator runs at 48 ksample/s.  This is a 4FSK
 * symbol/quality monitor, NOT a Horus Binary frame decoder.  The Horus
 * framing, symbol timing recovery, Golay, and ASN.1 layers remain pending.
 * Avoid dynamic allocation or UI calls in the real-time DSP callback. */
#define HORUS_FS 48000.0f
#define HORUS_BAUD 100u
#define HORUS_SPS 480u
#define HORUS_TONE_SPACING_HZ 270.0f
#define HORUS_PI2 6.2831853071795864769f

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static float published_rms, published_peak, published_center;
static uint32_t published_symbols, published_changes;
static uint8_t published_last;
static float dc, noise_var, peak, accum;
static uint32_t sample_count, symbols, changes;
static uint8_t last_symbol;
static bool have_last;

void horus_reset(void)
{
    portENTER_CRITICAL(&lock);
    published_rms = published_peak = published_center = 0;
    published_symbols = published_changes = 0;
    published_last = 0;
    portEXIT_CRITICAL(&lock);
    dc = noise_var = peak = accum = 0;
    sample_count = symbols = changes = 0;
    last_symbol = 0;
    have_last = false;
}

void horus_feed(float phase_delta)
{
    /* Smooth DC/receiver offset slowly relative to 100-baud symbols.
     * Keep signal metrics even without a valid packet. */
    dc += 0.00002f * (phase_delta - dc);
    float centered = phase_delta - dc;
    noise_var += 0.0002f * (centered * centered - noise_var);
    float deviation = fabsf(centered);
    if (deviation > peak) peak = deviation;

    /* First-pass integrate-and-dump at 100 baud. Timing is free-running:
     * this provides diagnostics, not reliable payload bits. */
    accum += centered;
    if (++sample_count < HORUS_SPS) return;
    float hz = (accum / (float)HORUS_SPS) * (HORUS_FS / HORUS_PI2);
    /* Four tones at -1.5, -0.5, +0.5, +1.5 spacing. */
    int sym = (int)floorf(hz / HORUS_TONE_SPACING_HZ + 2.0f);
    if (sym < 0) sym = 0;
    if (sym > 3) sym = 3;
    if (have_last && (uint8_t)sym != last_symbol) changes++;
    last_symbol = (uint8_t)sym;
    have_last = true;
    symbols++;
    accum = 0;
    sample_count = 0;

    if ((symbols % 10u) == 0u) {
        portENTER_CRITICAL(&lock);
        published_rms = sqrtf(fmaxf(noise_var, 0.0f)) * HORUS_FS / HORUS_PI2;
        published_peak = peak * HORUS_FS / HORUS_PI2;
        published_center = dc * HORUS_FS / HORUS_PI2;
        published_symbols = symbols;
        published_changes = changes;
        published_last = last_symbol;
        portEXIT_CRITICAL(&lock);
        peak = 0;
    }
}

void horus_status(char *dst, unsigned capacity)
{
    if (!dst || !capacity) return;
    float rms, pk, center;
    uint32_t n, transitions;
    uint8_t last;
    portENTER_CRITICAL(&lock);
    rms = published_rms;
    pk = published_peak;
    center = published_center;
    n = published_symbols;
    transitions = published_changes;
    last = published_last;
    portEXIT_CRITICAL(&lock);
    snprintf(dst, capacity, "HORUS 4FSK 100Bd: %.0fHz rms, %.0fHz peak, DC %.0fHz | sym %lu/%lu [%u] (no frames)",
             rms, pk, center, (unsigned long)transitions,
             (unsigned long)n, (unsigned)last);
}
