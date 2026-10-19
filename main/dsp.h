/*
 * dsp.h — IQ processing: demod (→ audio) + FFT spectrum (→ UI).
 *
 * Demodulates WFM / NFM / AM / USB / LSB / CW. Owns the sole ring consumer task. Rendering lives in ui.c, which reads the
 * spectrum snapshot exposed here.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the IQ-processing task. Call after sdr_stream_start(). */
void dsp_start(void);

/* Demod modes. Values 0/1 are unchanged from the old "nfm" flag, so existing
 * callers (bookmarks: .nfm = 0/1) keep working. */
enum {
    DSP_MODE_WFM = 0, /* wideband FM (broadcast)                          */
    DSP_MODE_NFM = 1, /* narrowband FM (NOAA / public safety voice)        */
    DSP_MODE_AM  = 2, /* AM envelope detector, carrier-referenced AGC      */
    DSP_MODE_USB = 3, /* upper sideband (dial = suppressed carrier)        */
    DSP_MODE_LSB = 4, /* lower sideband (dial = suppressed carrier)        */
    DSP_MODE_CW  = 5, /* CW, narrow filter, ~700 Hz pitch (dial = carrier  */
                      /* frequency minus pitch, like a normal SSB radio)   */
    DSP_MODE_RS41 = 6, /* Vaisala RS41 radiosonde, 4800 baud GFSK (see rs41.h) */
    DSP_MODE_FT8 = 7,  /* FT8 weak-signal digital mode: USB audio → ft8.c       */
    DSP_MODE_COUNT
};

/* Select demod mode (DSP_MODE_*). Thread-safe: the switch is applied by the
 * DSP task at the next block boundary, so it never races the filters. */
void dsp_set_mode(int mode);
int dsp_get_mode(void);

/* Hardware sample rate of the IQ stream. Supported: 1152000 (default, 1.15 MHz
 * view) and 288000 (HF: analysed and displayed at 96 kHz = 94 Hz per FFT bin).
 * Call this right after sdr_set_sample_rate(); applied at a block boundary. */
void dsp_set_sample_rate(uint32_t hz);

/* Width of the displayed spectrum in Hz (1152000 or 96000). */
uint32_t dsp_span_hz(void);

/* Short mode name ("WFM", "USB", ...). */
const char *dsp_mode_name(int mode);

/* Number of spectrum bins exposed (fftshifted: index 0 = lowest freq). */
int dsp_spectrum_bins(void);

/* Pointer to the latest spectrum frame (dB per bin), or NULL if none yet.
 * Valid until the next frame; read-only. */
const float *dsp_spectrum(void);

/* Squelch: mute audio when the signal level falls below `db`. Use a very low
 * value (e.g. -200) to disable. */
void dsp_set_squelch_db(float db);
float dsp_get_squelch_db(void);

/* Current smoothed signal level (dB, same scale as the squelch threshold). */
float dsp_signal_db(void);

#ifdef __cplusplus
}
#endif
