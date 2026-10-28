/*
 * dsp.c — IQ processing: WBFM demodulation (→ audio) + FFT spectrum (→ ui.c).
 *
 * iq_proc (core 1, high prio) is the SOLE ring consumer. Every block:
 *   - FM demod chain → audio_play()   (must be gap-free)
 *   - signal-level estimate for squelch
 * Every FFT_EVERY blocks it also runs one FFT, snapshotting a dB-per-bin frame
 * into a double buffer that ui.c reads. Rendering itself is in ui.c.
 *
 * FM demod @ 1.152 MS/s IQ:  /4 boxcar → 288 kHz → polar discriminator
 *                            → /6 boxcar → 48 kHz → 75 µs de-emphasis → int16.
 *
 * Baseband modes (AM / USB / LSB / CW) share one front end:
 *   1.152 MS/s: /4 boxcar → 288 kHz → /6 triangular (CIC-2) → 48 kHz complex.
 *   288 kS/s:   /2 boxcar → 144 kHz → /3 triangular (CIC-2) → 48 kHz complex —
 *               the /2 stage is essential: without it there is no anti-alias
 *               filter before the /3 stage and everything from 48 kHz to
 *               240 kHz offset folds into the audio.
 *   AM     : 5 kHz IIR low-pass on I/Q → envelope → carrier-referenced AGC.
 *   USB/LSB: DC block → Weaver: mix by ∓1.5 kHz → 255-tap FIR low-pass on I/Q
 *            → mix back, take real part (opposite sideband is rejected) → AGC.
 *   CW     : same Weaver, pitch 700 Hz, 6th-order Butterworth (~600 Hz wide).
 *   RS41   : 8 kHz IIR low-pass → FM discriminator at 48 kHz → rs41_feed().
 *            (audio = the data buzz, so you can hear when a sonde is there)
 * The dial frequency (tuner centre) is the suppressed carrier for SSB.
 */
#include "dsp.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_timer.h"

#include "sdr.h"
#include "audio.h"
#include "rs41.h"
#include "ft8.h"
#include "wspr.h"
#include "horus.h"
#include "horus_decoder.h"
#include "lora_aprs.h"

static const char *TAG = "dsp";

#define FFT_N 1024

#define FFT_EVERY     8
#define FFT_PER_FRAME 6

/* Demod parameters per mode, applied at 1.152 MS/s IQ:
 *   WFM (broadcast): /4 → 288 kHz IF, /6 → 48 kHz audio, 75 µs de-emphasis.
 *   NFM (voice):     /24 → 48 kHz IF, no further decim, no de-emphasis, more gain.
 */
static volatile int s_nfm;
static int s_decim_if = 4, s_decim_audio = 6;
static float s_inv_if = 1.0f / 4, s_inv_audio = 1.0f / 6;
static float s_deemph_a = 0.24f;
static float s_gain = 9000.0f;

/* ---- baseband modes (AM / USB / LSB / CW) ------------------------------- */

/* Set to 1 if USB and LSB turn out to be swapped on your hardware chain. */
#define DSP_SSB_SWAP 0

#define BB_RATE      48000.0f
#define SSB_PITCH_HZ 1500.0f /* Weaver centre: 300..2700 Hz → -1200..+1200 Hz */
#define SSB_CUT_HZ   1300.0f /* FIR cutoff (-6 dB point)                      */
/* FT8: same as USB but a wider passband (about 150..2950 Hz) for the 200..3000 Hz FT8 range */
#define FT8_PITCH_HZ 1550.0f
#define FT8_CUT_HZ   1400.0f
#define CW_PITCH_HZ  700.0f
#define CW_CUT_HZ    300.0f
#define AM_CUT_HZ    5000.0f

/* FM audio output filtering (post-discriminator, 48 kHz). WFM: a 15 kHz
 * low-pass removes the 19 kHz stereo pilot, the RDS subcarrier (57 kHz folds
 * to ~9 kHz through the /6 phase decimation) and the L-R subcarrier folded
 * into 10..25 kHz — without it broadcast FM sounds hissy/buzzy. NFM: 3.4 kHz
 * voice band-limit removes a large share of the discriminator noise. */
#define WFM_AUDIO_CUT_HZ 15000.0f
#define NFM_AUDIO_CUT_HZ 3400.0f

/* NFM de-emphasis: 300 µs one-pole (a = dt/(dt+RC) at 48 kHz ≈ 0.065).
 * Most FM voice links pre-emphasise; without the matching de-emphasis speech
 * sounds thin and hissy. Set to ~0.065f to enable, 0.0f for flat audio. */
#define NFM_DEEMPH_A 0.0f /* 0.065f = 300 µs de-emphasis on */

#define SSB_FIR_N 255 /* odd → linear phase, symmetric */

/* Audio levels (int16 full scale = 32767). Tune these on the device. */
#define SSB_AGC_TARGET 5000.0f /* nominal output amplitude                 */
#define AGC_FLOOR      1.0f    /* input-amplitude floor: limits idle gain  */
#define AGC_ATTACK     0.05f   /* per sample @48 kHz (~0.4 ms): fast, so onsets don't overshoot */
#define SSB_AGC_DECAY  0.00004f /* ~0.5 s                                  */
#define CW_AGC_DECAY   0.0002f  /* ~100 ms (faster: keyed signal)          */
#define AM_GAIN        14000.0f /* output for 100 % modulation             */
#define AM_FLOOR       3.0f     /* carrier floor: limits idle noise        */
#define AM_DC_ALPHA    0.0005f  /* carrier tracking (~40 ms)               */
#define DC_BLOCK_ALPHA 0.002f   /* complex DC blocker corner ~15 Hz        */

#define BQ_MAX 4
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1[2], z2[2]; /* state for channel 0 (I) and 1 (Q) */
} biquad_t;

static biquad_t s_bq[BQ_MAX];
static int s_bq_n;

static float s_fir_h[SSB_FIR_N];
static float s_fir_hist[2][2 * SSB_FIR_N];
static int s_fir_pos;

static int s_bb_mode;      /* DSP_MODE_AM / USB / LSB / CW */
static float s_bb_sgn;     /* +1 USB/CW, -1 LSB */
static float s_bb_w0;      /* mixer phase increment (rad/sample) */
static float s_osc_c = 1, s_osc_s, s_osc_wc = 1, s_osc_ws; /* rotating mixer oscillator */
static float s_agc_dec;
static float s_agc_env, s_am_dc;
static float s_dcI, s_dcQ;
static float s_fm_pa, s_fm_pb, s_rs_dc; /* RS41: previous IQ sample, audio DC tracker */
#ifndef RS41_CUT_HZ
#define RS41_CUT_HZ    6000.0f
#endif
#ifndef RS41_IF_SEC
#define RS41_IF_SEC    2 /* biquad sections of the IF filter (order = 2x) */
#endif
#define RS41_AUDIO_GAIN 18000.0f

/* front-end decimator state */
static float b_accI, b_accQ;
static int b_cnt4;
static int b_p6;
static float b_cI, b_nI, b_cQ, b_nQ;

/* /6 triangular (CIC-2) polyphase weights, sum over 6 samples = 1.
 * Each input contributes to the current output (C) and the next (N). */
static const float TRI_C[6] = {6 / 36.0f, 5 / 36.0f, 4 / 36.0f, 3 / 36.0f, 2 / 36.0f, 1 / 36.0f};
static const float TRI_N[6] = {0 / 36.0f, 1 / 36.0f, 2 / 36.0f, 3 / 36.0f, 4 / 36.0f, 5 / 36.0f};

static float s_window[FFT_N];
static float s_fft[2 * FFT_N];
static float s_acc[FFT_N];
static uint32_t s_acc_cnt;
static uint8_t s_blk[2 * FFT_N];

/* Double-buffered dB-per-bin spectrum (fftshifted) for the UI (lock-free). */
static float s_spec[2][FFT_N];
static volatile int s_ready = -1;

/* FM demod running state. */
static float d_accI, d_accQ;
static int d_cntIF;
static float d_pI, d_pQ;
static float d_accAud;
static int d_cntAud;
static float d_deemph;

/* Squelch / signal level. */
static volatile float s_sig_db = -120.0f;
static volatile float s_squelch_db = -200.0f; /* disabled by default */

/* Sample rate. 1.152 MS/s normally; 288 kS/s for HF (the RTL2832 cannot do
 * ~100 kS/s). At 288 kS/s the spectrum is taken from a /3 decimated 96 kS/s
 * stream, so the display spans 96 kHz. dsp_set_sample_rate() only posts a
 * request; the DSP task applies it at a block boundary. */
#define RATE_HI 1152000u
#define RATE_LO 288000u
static uint32_t s_rate = RATE_HI;        /* owned by the DSP task */
static volatile uint32_t s_req_rate;     /* 0 = no request */
static volatile uint32_t s_span = RATE_HI;
static int s_bb_d1 = 4;                  /* baseband stage-1 boxcar: 4 @ 1.152 MS/s, 2 @ 288 kS/s */
static float s_bb_inv1 = 0.25f;
static int s_bb_d2 = 6;                  /* stage-2 triangular decimation → 48 kHz */
static const float *s_bb_triC = TRI_C;   /* stage-2 polyphase weights (TRI_* /6 or W3* /3) */
static const float *s_bb_triN = TRI_N;

/* 288k -> 96k triangular (CIC-2) tap for the FFT */
static const float W3C[3] = {3 / 9.0f, 2 / 9.0f, 1 / 9.0f};
static const float W3N[3] = {0 / 9.0f, 1 / 9.0f, 2 / 9.0f};
static float l_cI, l_nI, l_cQ, l_nQ;
static int l_p;
static float s_lo_buf[2 * FFT_N];
static int s_lo_n;

/* Mode machine. dsp_set_mode() only posts a request; the DSP task applies it
 * at a block boundary so filter state is never reset under a running filter. */
static int s_mode;                 /* owned by the DSP task */
static volatile int s_mode_pub;    /* what dsp_get_mode() reports */
static volatile int s_req_mode = -1;

/* HORUS speaker PCM diagnostic, owned entirely by iq_proc_task. */
static uint64_t s_horus_pcm_sum_sq;
static uint64_t s_horus_pcm_count;
static uint32_t s_horus_pcm_peak;
static uint64_t s_horus_pcm_zero;


static inline float af_run(float x); /* FM audio low-pass, defined with the filter design below */

static void demod_fm(void)
{
    int16_t aud[FFT_N + 4]; /* worst case (NFM /24, audio /1): ~43/block */
    int naud = 0;
    float ifpow = 0;
    int nif = 0;
    bool open = s_sig_db >= s_squelch_db;

    int dif = s_decim_if, daud = s_decim_audio;
    float inv_if = s_inv_if, inv_aud = s_inv_audio, dea = s_deemph_a, gain = s_gain;

    for (int i = 0; i < FFT_N; i++) {
        float I = (float)s_blk[2 * i] - 127.4f;
        float Q = (float)s_blk[2 * i + 1] - 127.4f;
        d_accI += I;
        d_accQ += Q;
        if (++d_cntIF < dif) continue;
        float fi = d_accI * inv_if, fq = d_accQ * inv_if;
        d_accI = d_accQ = 0;
        d_cntIF = 0;

        ifpow += fi * fi + fq * fq;
        nif++;

        float dre = fi * d_pI + fq * d_pQ;
        float dim = fq * d_pI - fi * d_pQ;
        d_pI = fi;
        d_pQ = fq;
        float ang = atan2f(dim, dre);

        d_accAud += ang;
        if (++d_cntAud < daud) continue;
        float a = d_accAud * inv_aud;
        d_accAud = 0;
        d_cntAud = 0;

        float out;
        if (dea > 0) { d_deemph += dea * (a - d_deemph); out = d_deemph; }
        else out = a;
        out = af_run(out); /* audio low-pass: WFM 15 kHz (pilot/RDS/stereo), NFM 3.4 kHz (hiss) */
        int v = open ? (int)(out * gain) : 0;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        aud[naud++] = (int16_t)v;
    }
    if (naud) audio_play(aud, naud);

    if (nif) {
        float lvl = 10.0f * log10f(ifpow / nif + 1e-6f);
        s_sig_db += 0.1f * (lvl - s_sig_db); /* smooth */
    }
}

static void fft_block(void)
{
    for (int i = 0; i < FFT_N; i++) {
        float wi = s_window[i];
        s_fft[2 * i] = ((float)s_blk[2 * i] - 127.4f) * wi;
        s_fft[2 * i + 1] = ((float)s_blk[2 * i + 1] - 127.4f) * wi;
    }
    dsps_fft2r_fc32(s_fft, FFT_N);
    dsps_bit_rev_fc32(s_fft, FFT_N);
    for (int k = 0; k < FFT_N; k++) {
        float re = s_fft[2 * k], im = s_fft[2 * k + 1];
        s_acc[k] += re * re + im * im;
    }
    s_acc_cnt++;
}

/* 288 kS/s mode: FFT of 1024 consecutive samples of the 96 kS/s tap. */
static void fft_lo(void)
{
    for (int i = 0; i < FFT_N; i++) {
        float wi = s_window[i];
        s_fft[2 * i] = s_lo_buf[2 * i] * wi;
        s_fft[2 * i + 1] = s_lo_buf[2 * i + 1] * wi;
    }
    dsps_fft2r_fc32(s_fft, FFT_N);
    dsps_bit_rev_fc32(s_fft, FFT_N);
    for (int k = 0; k < FFT_N; k++) {
        float re = s_fft[2 * k], im = s_fft[2 * k + 1];
        s_acc[k] += re * re + im * im;
    }
    s_acc_cnt++;
}

static void snapshot(void)
{
    int w = (s_ready + 1) & 1;
    float *o = s_spec[w];
    /* /3 decimation averages noise power down by 3 (4.77 dB): add it back so the
     * noise floor sits at the same level as at 1.152 MS/s and the display scale holds. */
    const float off = (s_rate == RATE_LO) ? 4.77f : 0.0f;
    for (int i = 0; i < FFT_N; i++) {
        int bin = (i + FFT_N / 2) & (FFT_N - 1); /* fftshift: DC to centre */
        o[i] = 10.0f * log10f(s_acc[bin] / (float)s_acc_cnt + 1e-6f) + off;
    }
    s_ready = w;
    memset(s_acc, 0, sizeof(s_acc));
    s_acc_cnt = 0;
}

static void process_block(void);
const char *dsp_mode_name(int mode);

/* Called after every block: runs the spectrum FFT when due. */
static void fft_service(int *blocks)
{
    if (s_rate == RATE_LO) {
        if (s_lo_n >= FFT_N) { /* a full 1024-sample window of the 96 kS/s stream is ready */
            fft_lo();
            s_lo_n = 0;
            if (s_acc_cnt >= FFT_PER_FRAME) snapshot();
        }
        return;
    }
    if (++*blocks >= FFT_EVERY) {
        *blocks = 0;
        fft_block();
        if (s_acc_cnt >= FFT_PER_FRAME) snapshot();
    }
}

static void iq_proc_task(void *arg)
{
    (void)arg;
    size_t have = 0;
    int blocks = 0;
    int64_t busy_us = 0, win0 = esp_timer_get_time();
    while (1) {
        int64_t now = esp_timer_get_time();
        if (now - win0 >= 5000000) { /* demod load over the last 5 s (<100 % or audio drops) */
            ESP_LOGI(TAG, "mode %s: demod load %d %%", dsp_mode_name(s_mode), (int)(busy_us * 100 / (now - win0)));
            if (s_mode == DSP_MODE_HORUS && s_horus_pcm_count) {
                float rms = sqrtf((float)((double)s_horus_pcm_sum_sq / (double)s_horus_pcm_count));
                ESP_LOGI(TAG, "HORUS_PCM rms=%.1f peak=%lu zero=%llu/%llu",
                         (double)rms, (unsigned long)s_horus_pcm_peak,
                         (unsigned long long)s_horus_pcm_zero,
                         (unsigned long long)s_horus_pcm_count);
            }
            s_horus_pcm_sum_sq = 0;
            s_horus_pcm_count = 0;
            s_horus_pcm_peak = 0;
            s_horus_pcm_zero = 0;
            busy_us = 0; win0 = now;
        }
        if (lora_aprs_enabled()) {
            have = 0;
            /* No demodulation, decoder feeds or audio while LoRa owns RX. */
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        size_t n = sdr_read(s_blk + have, sizeof(s_blk) - have);
        have += n;
        if (have < sizeof(s_blk)) {
            if (n == 0) vTaskDelay(1);
            continue;
        }
        have = 0;
        int64_t t0 = esp_timer_get_time();
        process_block();
        busy_us += esp_timer_get_time() - t0;
        fft_service(&blocks);
    }
}

void dsp_start(void)
{
    dsps_fft2r_init_fc32(NULL, FFT_N);
    dsps_wind_hann_f32(s_window, FFT_N);
    memset(s_acc, 0, sizeof(s_acc));
    ESP_LOGI(TAG, "demod (WFM/NFM/AM/USB/LSB/CW) + FFT %d starting", FFT_N);
    xTaskCreatePinnedToCore(iq_proc_task, "iq_proc", 16384, NULL, 12, NULL, 1);
}

int dsp_spectrum_bins(void) { return FFT_N; }

const float *dsp_spectrum(void)
{
    int r = s_ready;
    return (r < 0) ? NULL : s_spec[r];
}

void dsp_set_squelch_db(float db) { s_squelch_db = db; }
float dsp_get_squelch_db(void) { return s_squelch_db; }
float dsp_signal_db(void) { return s_sig_db; }

/* ---- filter design -------------------------------------------------------- */

/* Butterworth low-pass as `nsec` cascaded RBJ biquads (order 2*nsec), written
 * into a caller-supplied array so independent filters don't share state. */
static void design_butter_lp_to(biquad_t *bq, int *pn, int nsec, float fc, float fs)
{
    *pn = nsec;
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float cw = cosf(w0), sw = sinf(w0);
    for (int k = 0; k < nsec; k++) {
        float th = (float)M_PI * (2 * k + 1) / (4.0f * nsec);
        float q = 1.0f / (2.0f * cosf(th));
        float alpha = sw / (2.0f * q);
        float a0 = 1.0f + alpha;
        biquad_t *b = &bq[k];
        b->b0 = (1.0f - cw) * 0.5f / a0;
        b->b1 = (1.0f - cw) / a0;
        b->b2 = b->b0;
        b->a1 = -2.0f * cw / a0;
        b->a2 = (1.0f - alpha) / a0;
        memset(b->z1, 0, sizeof b->z1);
        memset(b->z2, 0, sizeof b->z2);
    }
}

static void design_butter_lp(int nsec, float fc, float fs)
{
    design_butter_lp_to(s_bq, &s_bq_n, nsec, fc, fs);
}

/* FM audio low-pass (post-discriminator, 48 kHz). Independent of s_bq so it
 * survives mode switches of the baseband modes. WFM: kills the 19 kHz pilot,
 * the RDS subcarrier (57 kHz aliases to ~9 kHz!) and the L-R subcarrier that
 * the /6 phase boxcar folds into 10..25 kHz. NFM: voice band-limit → less hiss. */
static biquad_t s_af[4];
static int s_af_n;
static inline float bq_run_buf(biquad_t *b, float x); /* defined below */

static inline float af_run(float x)
{
    for (int k = 0; k < s_af_n; k++) x = bq_run_buf(&s_af[k], x);
    return x;
}

static inline float bq_run_buf(biquad_t *b, float x)
{
    float y = b->b0 * x + b->z1[0];
    b->z1[0] = b->b1 * x - b->a1 * y + b->z2[0];
    b->z2[0] = b->b2 * x - b->a2 * y;
    return y;
}

static inline float bq_run(int ch, float x)
{
    for (int k = 0; k < s_bq_n; k++) {
        biquad_t *b = &s_bq[k];
        float y = b->b0 * x + b->z1[ch];
        b->z1[ch] = b->b1 * x - b->a1 * y + b->z2[ch];
        b->z2[ch] = b->b2 * x - b->a2 * y;
        x = y;
    }
    return x;
}

/* Hamming-windowed sinc low-pass, unity DC gain. */
static void design_fir_lp(float fc, float fs)
{
    const int M = (SSB_FIR_N - 1) / 2;
    float wc = 2.0f * fc / fs, sum = 0;
    for (int n = 0; n < SSB_FIR_N; n++) {
        float x = (float)(n - M);
        float sinc = (x == 0) ? 1.0f : sinf((float)M_PI * wc * x) / ((float)M_PI * wc * x);
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n / (SSB_FIR_N - 1));
        s_fir_h[n] = wc * sinc * w;
        sum += s_fir_h[n];
    }
    for (int n = 0; n < SSB_FIR_N; n++) s_fir_h[n] /= sum;
    memset(s_fir_hist, 0, sizeof s_fir_hist);
    s_fir_pos = 0;
}

/* -O3 regardless of the project's (debug) optimisation level — this is the
 * hottest loop: 2 x 255 MACs per 48 kHz sample. */
__attribute__((optimize("O3")))
static float fir_run(int ch, float x)
{
    float *h = s_fir_hist[ch];
    h[s_fir_pos] = x;
    h[s_fir_pos + SSB_FIR_N] = x;
    const float *w = h + s_fir_pos + 1; /* oldest .. newest */
    /* 4 independent accumulators: a single chain is latency-bound (~4 cycles per
     * fmadd); this keeps the FPU busy. */
    float a0 = 0, a1 = 0, a2 = 0, a3 = 0;
    int i = 0;
    for (; i + 4 <= SSB_FIR_N; i += 4) {
        a0 += s_fir_h[i] * w[i];
        a1 += s_fir_h[i + 1] * w[i + 1];
        a2 += s_fir_h[i + 2] * w[i + 2];
        a3 += s_fir_h[i + 3] * w[i + 3];
    }
    for (; i < SSB_FIR_N; i++) a0 += s_fir_h[i] * w[i];
    return (a0 + a1) + (a2 + a3);
}

static inline void fir_advance(void)
{
    if (++s_fir_pos >= SSB_FIR_N) s_fir_pos = 0;
}

/* ---- baseband demodulation (AM / USB / LSB / CW) --------------------------- */

static inline int16_t clip16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static void demod_bb(void)
{
    int16_t aud[FFT_N / 4]; /* 1024 in: ~43 out at 1.152 MS/s, ~171 at 288 kS/s */
    int naud = 0;
    float ifpow = 0;
    int nif = 0;
    bool open = s_sig_db >= s_squelch_db;
    const int mode = s_bb_mode;
    const float sgn = s_bb_sgn;
    const int d1 = s_bb_d1;
    const float inv1 = s_bb_inv1;
    const float owc = s_osc_wc, ows = s_osc_ws;
    float oc = s_osc_c, os = s_osc_s;

    for (int i = 0; i < FFT_N; i++) {
        float I = (float)s_blk[2 * i] - 127.4f;
        float Q = (float)s_blk[2 * i + 1] - 127.4f;

        /* stage 1: boxcar → 288 kHz (/4 at 1.152 MS/s) or → 144 kHz (/2 at 288 kS/s) */
        b_accI += I;
        b_accQ += Q;
        if (++b_cnt4 < d1) continue;
        float fi = b_accI * inv1, fq = b_accQ * inv1;
        b_accI = b_accQ = 0;
        b_cnt4 = 0;

        /* stage 2: triangular (CIC-2) → 48 kHz (÷6 @ 1.152 MS/s, ÷3 @ 288 kS/s) */
        int p = b_p6;
        b_cI += fi * s_bb_triC[p];
        b_nI += fi * s_bb_triN[p];
        b_cQ += fq * s_bb_triC[p];
        b_nQ += fq * s_bb_triN[p];
        if (++b_p6 < s_bb_d2) continue;
        b_p6 = 0;
        float zI = b_cI, zQ = b_cQ;
        b_cI = b_nI; b_nI = 0;
        b_cQ = b_nQ; b_nQ = 0;

        ifpow += zI * zI + zQ * zQ;
        nif++;

        float out = 0;
        if (mode == DSP_MODE_RS41 || mode == DSP_MODE_HORUS) {
            /* Horus: remove RTL2832 centre-frequency DC spur before 4FSK modem.
             * Do not alter RS41 path or the waterfall. */
            if (mode == DSP_MODE_HORUS) {
                s_dcI += 0.001f * (zI - s_dcI);
                s_dcQ += 0.001f * (zQ - s_dcQ);
                zI -= s_dcI;
                zQ -= s_dcQ;
            }
            float a = bq_run(0, zI), b = bq_run(1, zQ);
            float dre = a * s_fm_pa + b * s_fm_pb;
            float dim = b * s_fm_pa - a * s_fm_pb;
            s_fm_pa = a;
            s_fm_pb = b;
            float d = atan2f(dim, dre); /* rad/sample: ±2.4 kHz dev = ±0.31 */
            if (mode == DSP_MODE_RS41) rs41_feed(d);
            else { horus_feed(d); horus_decoder_feed_iq(a,b); }
            s_rs_dc += 0.001f * (d - s_rs_dc);
            out = (d - s_rs_dc) * RS41_AUDIO_GAIN;
        } else if (mode == DSP_MODE_AM) {
            float a = bq_run(0, zI), b = bq_run(1, zQ);
            float mag = sqrtf(a * a + b * b);
            s_am_dc += AM_DC_ALPHA * (mag - s_am_dc);
            float den = s_am_dc > AM_FLOOR ? s_am_dc : AM_FLOOR;
            out = (mag - s_am_dc) / den * AM_GAIN;
        } else {
            /* kill the RTL-SDR DC spike at the dial frequency */
            s_dcI += DC_BLOCK_ALPHA * (zI - s_dcI);
            s_dcQ += DC_BLOCK_ALPHA * (zQ - s_dcQ);
            zI -= s_dcI;
            zQ -= s_dcQ;

            /* Weaver: shift the wanted sideband to be centred on DC … */
            float nc = oc * owc - os * ows; /* rotate oscillator (no sinf/cosf) */
            os = os * owc + oc * ows;
            oc = nc;
            float c = oc, sn = os;
            float a = zI * c + sgn * zQ * sn;
            float b = zQ * c - sgn * zI * sn;

            /* … low-pass it … */
            if (mode == DSP_MODE_CW) {
                a = bq_run(0, a);
                b = bq_run(1, b);
            } else {
                float ya = fir_run(0, a), yb = fir_run(1, b);
                fir_advance();
                a = ya;
                b = yb;
            }

            /* … and shift back up, real part = audio. */
            float y = a * c - sgn * b * sn;

            if (mode == DSP_MODE_FT8) ft8_feed(y); /* decoders get the audio before the AGC */
            else if (mode == DSP_MODE_WSPR) wspr_feed(y);

            /* feed-forward peak AGC */
            float ay = fabsf(y);
            s_agc_env += (ay > s_agc_env ? AGC_ATTACK : s_agc_dec) * (ay - s_agc_env);
            out = y * (SSB_AGC_TARGET / (s_agc_env + AGC_FLOOR));
        }
        /* soft limiter above 20000 instead of hard clipping */
        {
            float ao = fabsf(out);
            if (ao > 20000.0f) {
                float e = ao - 20000.0f;
                ao = 20000.0f + 12000.0f * e / (e + 12000.0f);
                out = out < 0 ? -ao : ao;
            }
        }
        aud[naud++] = open ? clip16(out) : 0;
    }
    /* renormalise the oscillator once per block so it never drifts */
    {
        float m = 1.0f / sqrtf(oc * oc + os * os);
        s_osc_c = oc * m;
        s_osc_s = os * m;
    }
    if (mode == DSP_MODE_HORUS) {
        for (int j = 0; j < naud; ++j) {
            int32_t v = aud[j];
            uint32_t mag = (uint32_t)(v < 0 ? -v : v);
            s_horus_pcm_sum_sq += (uint64_t)mag * mag;
            s_horus_pcm_count++;
            if (mag > s_horus_pcm_peak) s_horus_pcm_peak = mag;
            if (v == 0) s_horus_pcm_zero++;
        }
    }
    if (naud) audio_play(aud, naud);

    if (nif) {
        float lvl = 10.0f * log10f(ifpow / nif + 1e-6f);
        s_sig_db += 0.1f * (lvl - s_sig_db); /* smooth */
    }
}

/* ---- mode switching (runs on the DSP task) --------------------------------- */

static void apply_mode(int m)
{
    /* Baseband decimation to 48 kHz, always as boxcar → triangular (CIC-2).
     * At 288 kS/s the /2 boxcar is essential anti-alias for the /3 stage:
     * without it everything from 48 kHz to 240 kHz offset aliases into the
     * audio (FT8/WSPR live at 288 kS/s — this was the "dirty HF audio"). */
    if (s_rate == RATE_HI) {
        s_bb_d1 = 4; s_bb_d2 = 6;
        s_bb_triC = TRI_C; s_bb_triN = TRI_N;
    } else {
        s_bb_d1 = 2; s_bb_d2 = 3;
        s_bb_triC = W3C; s_bb_triN = W3N;
    }
    s_bb_inv1 = 1.0f / s_bb_d1;
    /* reset every demod/filter state to avoid a transient */
    d_accI = d_accQ = d_accAud = 0;
    d_cntIF = d_cntAud = 0;
    d_pI = d_pQ = d_deemph = 0;
    b_accI = b_accQ = 0; b_cnt4 = 0; b_p6 = 0;
    b_cI = b_nI = b_cQ = b_nQ = 0;
    s_dcI = s_dcQ = 0;
    s_agc_env = 0; s_am_dc = 0;
    s_fm_pa = s_fm_pb = s_rs_dc = 0;
    s_osc_c = 1; s_osc_s = 0;

    switch (m) {
    case DSP_MODE_NFM:
        s_decim_if = (int)(s_rate / 48000u); s_decim_audio = 1; /* 48 kHz IF */
        s_deemph_a = NFM_DEEMPH_A; s_gain = 18000.0f;
        design_butter_lp_to(s_af, &s_af_n, 2, NFM_AUDIO_CUT_HZ, BB_RATE);
        break;
    case DSP_MODE_AM:
        design_butter_lp(2, AM_CUT_HZ, BB_RATE);
        s_bb_sgn = 1; s_bb_w0 = 0;
        break;
    case DSP_MODE_HORUS:
        design_butter_lp(3, 2400.0f, BB_RATE);
        s_bb_sgn = 1; s_bb_w0 = 0;
        horus_reset();
        horus_decoder_reset();
        break;
    case DSP_MODE_RS41:
        design_butter_lp(RS41_IF_SEC, RS41_CUT_HZ, BB_RATE);
        s_bb_sgn = 1; s_bb_w0 = 0;
        rs41_reset();
        break;
    case DSP_MODE_USB:
    case DSP_MODE_LSB:
    case DSP_MODE_FT8:
    case DSP_MODE_WSPR: {
        float sg = (m == DSP_MODE_LSB) ? -1.0f : 1.0f; /* FT8 and WSPR are USB */
        if (DSP_SSB_SWAP) sg = -sg;
        bool dig = (m == DSP_MODE_FT8 || m == DSP_MODE_WSPR); /* wide passband for the digital modes */
        float pitch = dig ? FT8_PITCH_HZ : SSB_PITCH_HZ;
        float cut = dig ? FT8_CUT_HZ : SSB_CUT_HZ;
        s_bb_sgn = sg;
        s_bb_w0 = 2.0f * (float)M_PI * pitch / BB_RATE;
        s_osc_wc = cosf(s_bb_w0); s_osc_ws = sinf(s_bb_w0);
        design_fir_lp(cut, BB_RATE);
        s_agc_dec = SSB_AGC_DECAY;
        if (m == DSP_MODE_FT8) ft8_reset();
        if (m == DSP_MODE_WSPR) wspr_reset();
        break;
    }
    case DSP_MODE_CW:
        s_bb_sgn = 1;
        s_bb_w0 = 2.0f * (float)M_PI * CW_PITCH_HZ / BB_RATE;
        s_osc_wc = cosf(s_bb_w0); s_osc_ws = sinf(s_bb_w0);
        design_butter_lp(3, CW_CUT_HZ, BB_RATE);
        s_agc_dec = CW_AGC_DECAY;
        break;
    case DSP_MODE_WFM:
    default:
        m = DSP_MODE_WFM;
        s_decim_if = (int)(s_rate / 288000u); s_decim_audio = 6; /* 288 kHz IF → 48 kHz audio */
        s_deemph_a = 0.24f; s_gain = 9000.0f;
        design_butter_lp_to(s_af, &s_af_n, 2, WFM_AUDIO_CUT_HZ, BB_RATE);
        break;
    }
    if (m <= DSP_MODE_NFM) {
        s_inv_if = 1.0f / s_decim_if;
        s_inv_audio = 1.0f / s_decim_audio;
    } else {
        s_bb_mode = m;
    }
    s_nfm = (m == DSP_MODE_NFM);
    s_mode = m;
}

/* 288 kS/s → 96 kS/s tap feeding the FFT (runs for every demod mode). */
static void lo_tap(void)
{
    for (int i = 0; i < FFT_N; i++) {
        float I = (float)s_blk[2 * i] - 127.4f;
        float Q = (float)s_blk[2 * i + 1] - 127.4f;
        int p = l_p;
        l_cI += I * W3C[p];
        l_nI += I * W3N[p];
        l_cQ += Q * W3C[p];
        l_nQ += Q * W3N[p];
        if (++l_p < 3) continue;
        l_p = 0;
        if (s_lo_n < FFT_N) {
            s_lo_buf[2 * s_lo_n] = l_cI;
            s_lo_buf[2 * s_lo_n + 1] = l_cQ;
            s_lo_n++;
        }
        l_cI = l_nI; l_nI = 0;
        l_cQ = l_nQ; l_nQ = 0;
    }
}

static void process_block(void)
{
    uint32_t rr = s_req_rate;
    if (rr) {
        s_req_rate = 0;
        if (rr != s_rate) {
            s_rate = rr;
            s_span = (rr == RATE_LO) ? 96000u : rr;
            s_lo_n = 0;
            l_cI = l_nI = l_cQ = l_nQ = 0;
            l_p = 0;
            memset(s_acc, 0, sizeof s_acc);
            s_acc_cnt = 0;
            apply_mode(s_mode); /* decimation factors depend on the rate */
        }
    }
    if (s_rate == RATE_LO) lo_tap();
    int req = s_req_mode;
    if (req >= 0) {
        s_req_mode = -1;
        if (req != s_mode) apply_mode(req);
    }
    if (s_mode <= DSP_MODE_NFM) demod_fm();
    else demod_bb();
}

void dsp_set_sample_rate(uint32_t hz)
{
    if (hz != RATE_HI && hz != RATE_LO) return;
    s_req_rate = hz;
}

uint32_t dsp_span_hz(void) { return s_span; }

void dsp_set_mode(int mode)
{
    if (mode < 0 || mode >= DSP_MODE_COUNT) mode = DSP_MODE_WFM;
    s_mode_pub = mode;
    s_req_mode = mode;
}

int dsp_get_mode(void) { return s_mode_pub; }

const char *dsp_mode_name(int mode)
{
    static const char *const N[DSP_MODE_COUNT] = {"WFM", "NFM", "AM", "USB", "LSB", "CW", "RS41", "FT8", "WSPR", "HORUS"};
    return (mode >= 0 && mode < DSP_MODE_COUNT) ? N[mode] : "?";
}
