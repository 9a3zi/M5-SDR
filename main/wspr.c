/*
 * wspr.c — WSPR decoder. See wspr.h.
 *
 * Pipeline
 *   12 kHz audio → every 1024 samples (85 ms) an 8192-sample
 *   (= one symbol) FFT, zero-padded to 16384 (0.73 Hz bins), band 1300..1700 Hz,
 *   power normalised by the frame's median (≈ noise = 1) → rolling spectrogram.
 *   A symbol lasts exactly 8 frames, so a transmission starting at frame t0 has
 *   symbol i in frame t0 + 8 i and tone k in bin b0 + 2 k.
 *   Sync search: for every (t0, b0) correlate the known sync vector with the
 *   odd-minus-even tone power (data independent).  Peaks are refined (time ±1
 *   frame, frequency ±1 bin, drift) and soft symbols are decoded with a Fano
 *   sequential decoder (the K=32 code is far too long for Viterbi).
 */
#include "wspr.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "fft/kiss_fftr.h"

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psk.h"
#include "sdr.h"
#define WSPR_YIELD() vTaskDelay(1)
static inline int64_t now_us(void) { return esp_timer_get_time(); }
#else
#include <time.h>
#define WSPR_YIELD() ((void)0)
static inline int64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}
#endif

static const char *TAG __attribute__((unused)) = "wspr";

/* ---- parameters ------------------------------------------------------------ */

#define SR          12000
#define WIN         8192                  /* one symbol = 0.6827 s                */
#define NFFT        16384                 /* zero padded: 0.7324 Hz bins          */
#define HOP         1024                  /* frame step 85.3 ms; 8 frames = 1 symbol */
#define SYM_FR      (WIN / HOP)           /* 8                                    */
#define NSYM        162
#define SPAN_FR     ((NSYM - 1) * SYM_FR) /* 1288 frames between first and last symbol */
#define F_LO_HZ     1300                  /* analysed audio band (WSPR is 1400..1600 Hz) */
#define F_HI_HZ     1700
#define BIN_HZ      ((float)SR / NFFT)
#define B_LO        (F_LO_HZ * NFFT / SR) /* integer arithmetic: must be a constant expression */
#define B_HI        (F_HI_HZ * NFFT / SR)
#define NB          (B_HI - B_LO)         /* 547 bins                             */
#define NB0         (NB - 6)              /* usable lowest-tone bins              */
#define FR          1600                  /* frames kept (136 s)                  */
#define SCAN_EVERY  64                    /* frames between sync scans (5.5 s)    */
#define SCAN_BACK   16                    /* overlap with the previous scan       */
#define SCAN_MAX    160                   /* starts examined per scan (max)       */
#define MAX_PEAKS   10

#define RING_N      262144                /* 21.8 s of 12 kHz audio: survives a starved task */
#define MSG_N       24

#ifndef WSPR_MTHR
#define WSPR_MTHR   0.15f                 /* sync-correlation threshold            */
#endif
#ifndef WSPR_MU
#define WSPR_MU     1.4f                  /* design signal level of the soft metric */
#endif
#ifndef WSPR_DELTA
#define WSPR_DELTA  3.0f                  /* Fano threshold step                   */
#endif
#ifndef WSPR_MAXCYC
#define WSPR_MAXCYC 400000u               /* Fano cycle limit per candidate        */
#endif
#ifndef WSPR_QMIN
#define WSPR_QMIN   3.0f                  /* min soft-symbol agreement (z-score)   */
#endif
#ifndef WSPR_SNR_K
#define WSPR_SNR_K  (-32.3f)              /* dB: tone/noise-bin ratio → SNR in 2500 Hz (rect. window) */
#endif

#define NORM_TARGET 0.03f
#define NORM_ALPHA  (1.0f / (SR * 3.0f))

#define POLY1 0xf2d05351u
#define POLY2 0xe4613c47u

/* Sync vector (LSB of each channel symbol), per the WSPR protocol description. */
static const uint8_t SYNC[NSYM] = {
    1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,
    0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,
    1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
    1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0};

/* ---- allocation / helpers ------------------------------------------------------ */

static void *big_alloc(size_t n)
{
#ifdef ESP_PLATFORM
    void *p = heap_caps_calloc(1, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) return p;
#endif
    return calloc(1, n);
}

/* Hot FFT buffers go to internal RAM if that leaves plenty free, else PSRAM. */
static void *fast_alloc(size_t n)
{
#ifdef ESP_PLATFORM
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > n + 300 * 1024) {
        void *p = heap_caps_calloc(1, n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (p) return p;
    }
#endif
    return big_alloc(n);
}

static void copy_str(char *dst, size_t dst_size, const char *src)
{
    size_t n = strlen(src);
    if (n > dst_size - 1) n = dst_size - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static inline int parity32(uint32_t x) { return __builtin_parity(x); }

/* ---- producer side (DSP task) --------------------------------------------------- */

static float *s_ring;
static volatile uint32_t s_head, s_tail;
static volatile int s_reset_req;
static volatile uint32_t s_drops; /* audio samples lost because the consumer was too slow */
static float s_acc4;
static int s_n4;
static float s_pw;
static uint32_t s_norm_n;

void wspr_feed(float y)
{
    s_acc4 += y;
    if (++s_n4 < 4) return;
    float x = s_acc4 * 0.25f;
    s_acc4 = 0;
    s_n4 = 0;
    if (!s_ring) return;

    float a = (s_norm_n < (uint32_t)(SR * 3)) ? 1.0f / (float)(s_norm_n + 1) : NORM_ALPHA;
    s_norm_n++;
    s_pw += a * (x * x - s_pw);
    x *= NORM_TARGET / sqrtf(s_pw + 1e-12f);
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    uint32_t h = s_head;
    if (h - s_tail >= RING_N) { /* consumer far behind: drop (a short hole is better than losing 2 minutes) */
        s_drops++;
        return;
    }
    s_ring[h & (RING_N - 1)] = x;
    __atomic_store_n(&s_head, h + 1, __ATOMIC_RELEASE);
}

void wspr_reset(void)
{
    s_acc4 = 0;
    s_n4 = 0;
    s_pw = 0;
    s_norm_n = 0;
    s_reset_req = 1;
}

/* ---- spectrogram ------------------------------------------------------------------ */

static int s_inited;
static float *P;                    /* [FR][NB] noise-normalised power           */
static float *s_hist;               /* last WIN audio samples                    */
static float *s_win, *s_td;
static kiss_fft_cpx *s_fd;
static kiss_fftr_cfg s_cfg;
static void *s_cfg_mem;
static float *s_M;                  /* sync metric [SCAN_MAX + SCAN_BACK][NB0]   */
static float s_tmp[NB];

static int64_t s_samples;           /* audio samples consumed since reset        */
static int64_t s_frames;            /* frames produced (newest = s_frames - 1)   */
static int64_t s_next_start;        /* first start frame not yet scanned         */
static int64_t s_next_scan_frame;
static uint32_t s_scans, s_cands, s_fano_ok;
static float s_best_m;
static int s_in_scan;       /* a scan is running (drain() must not start another) */
static int64_t s_scan_ref;  /* frame count when the running scan started */
static int s_st_state, s_st_pct;
static float s_st_speed;

static inline float *row(int64_t k) { return P + (size_t)(k % FR) * NB; }

static void init_once(void)
{
    if (s_inited) return;
    P = big_alloc((size_t)FR * NB * sizeof(float));
    s_hist = fast_alloc(WIN * sizeof(float));
    s_win = fast_alloc(WIN * sizeof(float));
    s_td = fast_alloc(NFFT * sizeof(float));
    s_fd = fast_alloc((NFFT / 2 + 1) * sizeof(kiss_fft_cpx));
    s_M = big_alloc((size_t)(SCAN_MAX + SCAN_BACK + 1) * NB0 * sizeof(float));
    /* Rectangular window, exactly one symbol long: the four tones are 1/T apart, so they are
     * orthogonal. (A Hann window widens the main lobe to +-2 bins and smears adjacent tones
     * together, which cost ~20 dB of decoding sensitivity.) */
    for (int i = 0; i < WIN; i++) s_win[i] = 1.0f;
    size_t sz = 0;
    kiss_fftr_alloc(NFFT, 0, 0, &sz);
    s_cfg_mem = fast_alloc(sz);
    s_cfg = kiss_fftr_alloc(NFFT, 0, s_cfg_mem, &sz);
    s_inited = (P && s_hist && s_win && s_td && s_fd && s_M && s_cfg) ? 1 : 0;
    if (!s_inited) ESP_LOGE(TAG, "out of memory");
}

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static void make_frame(void)
{
    for (int i = 0; i < WIN; i++) s_td[i] = s_hist[i] * s_win[i];
    memset(s_td + WIN, 0, (NFFT - WIN) * sizeof(float));
    kiss_fftr(s_cfg, s_td, s_fd);

    float *r = row(s_frames);
    for (int b = 0; b < NB; b++) {
        const kiss_fft_cpx *c = &s_fd[B_LO + b];
        r[b] = c->r * c->r + c->i * c->i;
        s_tmp[b] = r[b];
    }
    qsort(s_tmp, NB, sizeof s_tmp[0], cmp_f);
    float noise = s_tmp[NB / 2] / 0.693f; /* exponential noise: mean = median / ln 2 */
    if (noise < 1e-20f) noise = 1e-20f;
    float inv = 1.0f / noise;
    for (int b = 0; b < NB; b++) r[b] *= inv;
    s_frames++;
}

/* ---- Fano sequential decoder ----------------------------------------------------- */

typedef struct {
    uint32_t state;
    float gamma;
    float tm[2];
    uint8_t bit[2];
    uint8_t i;
} fnode_t;

static uint8_t s_perm[NSYM]; /* coded-bit index p → position j in the transmitted symbol sequence */

static void build_perm(void)
{
    int p = 0;
    for (int k = 0; k < 256; k++) {
        int j = 0;
        for (int l = 0; l < 8; l++) j |= ((k >> l) & 1) << (7 - l);
        if (j < NSYM) s_perm[p++] = (uint8_t)j;
    }
}

static inline void fnode_setup(fnode_t *n, int depth, const float (*mt)[2])
{
    float m[2];
    for (int b = 0; b < 2; b++) {
        uint32_t ns = (n->state << 1) | (uint32_t)b;
        int p1 = parity32(ns & POLY1), p2 = parity32(ns & POLY2);
        m[b] = mt[2 * depth][p1] + mt[2 * depth + 1][p2];
    }
    n->i = 0;
    if (depth >= 50) { /* tail: the encoder is flushed with zeros, only the 0 branch exists */
        n->bit[0] = 0; n->bit[1] = 1; n->tm[0] = m[0]; n->tm[1] = -1e30f;
        return;
    }
    if (m[1] > m[0]) { n->bit[0] = 1; n->bit[1] = 0; n->tm[0] = m[1]; n->tm[1] = m[0]; }
    else { n->bit[0] = 0; n->bit[1] = 1; n->tm[0] = m[0]; n->tm[1] = m[1]; }
}

/* mt[p][b]: metric of coded bit p having value b. Returns true on success, fills bits[50]. */
static bool fano_decode(const float (*mt)[2], float delta, uint32_t maxcyc, uint8_t *bits, uint32_t *cycles)
{
    static fnode_t nd[82];
    const int nbits = 81, ndata = 50;
    int d = 0;
    float T = 0;
    nd[0].state = 0;
    nd[0].gamma = 0;
    fnode_setup(&nd[0], 0, mt);
    for (uint32_t cyc = 1; cyc <= maxcyc; cyc++) {
        float ng = nd[d].gamma + nd[d].tm[nd[d].i];
        if (ng >= T) {
            if (nd[d].gamma < T + delta)
                while (ng >= T + delta) T += delta; /* tighten */
            nd[d + 1].state = (nd[d].state << 1) | nd[d].bit[nd[d].i];
            nd[d + 1].gamma = ng;
            d++;
            if (d == nbits) {
                for (int k = 0; k < ndata; k++) bits[k] = (uint8_t)((nd[k + 1].state) & 1u);
                *cycles = cyc;
                return true;
            }
            fnode_setup(&nd[d], d, mt);
            continue;
        }
        for (;;) { /* back up / loosen */
            if (d == 0 || nd[d - 1].gamma < T) {
                T -= delta;
                nd[d].i = 0;
                break;
            }
            d--;
            if (nd[d].i == 0 && d < ndata) {
                nd[d].i = 1;
                break;
            }
        }
    }
    *cycles = maxcyc;
    return false;
}

/* ---- message unpacking ---------------------------------------------------------------- */

static char call_char(int v) { return v < 10 ? (char)('0' + v) : (v < 36 ? (char)('A' + v - 10) : ' '); }

/* 50 bits → "CALL GRID PWR". Returns 1 = type 1 ok, 0 = invalid, 2 = extended type (not shown). */
static int unpack_msg(const uint8_t *bits, char *out, size_t out_size)
{
    uint32_t n = 0, m = 0;
    for (int i = 0; i < 28; i++) n = (n << 1) | bits[i];
    for (int i = 28; i < 50; i++) m = (m << 1) | bits[i];

    int power = (int)(m & 127u) - 64;
    uint32_t ng = m >> 7;
    if (power < 0 || power > 60) return 0;
    if (ng >= 32400) return 2; /* type 2 / 3 */
    if (power % 10 != 0 && power % 10 != 3 && power % 10 != 7) return 2;

    uint32_t t = n;
    int c[6];
    c[5] = (int)(t % 27) + 10; t /= 27;
    c[4] = (int)(t % 27) + 10; t /= 27;
    c[3] = (int)(t % 27) + 10; t /= 27;
    c[2] = (int)(t % 10);      t /= 10;
    c[1] = (int)(t % 36);      t /= 36;
    c[0] = (int)t;
    if (c[0] > 36) return 0;
    if (c[1] < 10 || c[1] > 35) return 0; /* second character must be a letter */
    char call[7];
    for (int i = 0; i < 6; i++) call[i] = call_char(c[i]);
    call[6] = '\0';
    bool sp = false; /* spaces only at the end (c[5]..c[3]) or the very start */
    for (int i = 3; i < 6; i++) {
        if (call[i] == ' ') sp = true;
        else if (sp) return 0;
    }
    const char *p = call;
    while (*p == ' ') p++;
    char callt[7];
    size_t k = 0;
    for (; *p && *p != ' '; p++) callt[k++] = *p;
    callt[k] = '\0';
    if (k < 3) return 0;

    uint32_t a = ng / 180, b = ng % 180;
    int v0 = (int)(179 - a) / 10, v2 = (int)(179 - a) % 10, v1 = (int)b / 10, v3 = (int)b % 10;
    if (v0 > 17 || v1 > 17) return 0;
    snprintf(out, out_size, "%s %c%c%d%d %d", callt, 'A' + v0, 'A' + v1, v2, v3, power);
    return 1;
}

/* ---- candidate scan ------------------------------------------------------------------- */

typedef struct {
    int64_t start; /* frame index of symbol 0 */
    int b0;        /* lowest tone bin (relative to B_LO) */
    float metric;
} peak_t;

static inline int path_bin(int b0, int drift, int i)
{
    /* linear drift of `drift` bins over the message, centred on the middle */
    float f = (float)drift * ((float)i - 80.5f) / 161.0f;
    return b0 + (int)lroundf(f);
}

/* A start frame is usable only if all 162 symbol rows exist and are still in the ring. */
static inline bool valid_start(int64_t t0)
{
    return t0 >= 0 && t0 >= s_frames - FR && t0 + SPAN_FR <= s_frames - 1;
}

/* sync correlation for one hypothesis; returns metric in [-1, 1] */
static float sync_metric(int64_t t0, int b0, int drift)
{
    if (!valid_start(t0)) return -1.0f;
    float acc = 0, tot = 0;
    for (int i = 0; i < NSYM; i++) {
        int b = path_bin(b0, drift, i);
        if (b < 0 || b > NB - 7) return -1.0f;
        const float *r = row(t0 + (int64_t)i * SYM_FR) + b;
        float ev = r[0] + r[4], od = r[2] + r[6];
        float d = od - ev;
        acc += SYNC[i] ? d : -d;
        tot += ev + od;
    }
    return tot > 0 ? acc / tot : 0;
}

/* ---- message list ---------------------------------------------------------------------- */

static wspr_msg_t s_msgs[MSG_N];
static uint32_t s_msg_total;
static uint32_t s_other;
static volatile uint32_t s_lockseq;
static bool s_grid_known;
static int64_t s_grid; /* sample index of a nominal "second 0 of an even minute" */

static struct {
    char text[WSPR_TEXT_MAX];
    float freq;
    int64_t t;
} s_recent[MSG_N];
static int s_recent_n;

static bool is_recent(const char *text, float freq, int64_t start)
{
    for (int i = 0; i < MSG_N && i < s_recent_n; i++)
        if (!strcmp(s_recent[i].text, text) && fabsf(s_recent[i].freq - freq) < 4.0f &&
            llabs(s_recent[i].t - start) < 100LL * SR)
            return true;
    return false;
}

static void add_message(const char *text, float freq, int snr, int drift, int64_t start)
{
    int i = s_recent_n % MSG_N;
    copy_str(s_recent[i].text, sizeof s_recent[i].text, text);
    s_recent[i].freq = freq;
    s_recent[i].t = start;
    s_recent_n++;

    if (!s_grid_known) { /* the first spot defines the 2-minute grid: transmissions start 1 s into the cycle */
        s_grid = start - SR;
        s_grid_known = true;
    }
    int64_t rel = (start - s_grid - SR) % (120LL * SR);
    if (rel >= 60LL * SR) rel -= 120LL * SR;
    if (rel < -60LL * SR) rel += 120LL * SR;

    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
    wspr_msg_t *m = &s_msgs[s_msg_total % MSG_N];
    m->seq = s_msg_total + 1;
    m->snr = (int16_t)snr;
    m->dt = (float)rel / SR;
    m->freq = freq;
    m->drift = (int8_t)drift;
    copy_str(m->text, sizeof m->text, text);
    s_msg_total++;
    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);

#ifdef ESP_PLATFORM
    { /* upload to PSK Reporter: RF freq = dial centre + audio tone */
        sdr_stats_t sst;
        sdr_get_stats(&sst);
        psk_spot_msg(text, (double)sst.center_freq + (double)freq, snr, "WSPR");
    }
#endif
}

int wspr_get_messages(wspr_msg_t *out, int max, wspr_status_t *st)
{
    for (int tries = 0; tries < 4; tries++) {
        uint32_t a = __atomic_load_n(&s_lockseq, __ATOMIC_ACQUIRE);
        if (a & 1) continue;
        uint32_t total = s_msg_total;
        int n = (int)(total < MSG_N ? total : MSG_N);
        if (n > max) n = max;
        for (int i = 0; i < n; i++) out[i] = s_msgs[(total - n + i) % MSG_N];
        wspr_status_t s = {0};
        s.total = total;
        s.other_types = s_other;
        s.grid_known = s_grid_known;
        s.buffered_s = (float)(s_frames > FR ? FR : s_frames) * HOP / SR;
        s.scans = s_scans;
        s.cands = s_cands;
        s.fano_ok = s_fano_ok;
        s.drops = s_drops;
        s.best_m = s_best_m;
        s.selftest_state = s_st_state;
        s.selftest_pct = s_st_pct;
        s.selftest_speed = s_st_speed;
        if (s_grid_known) {
            int64_t pos = (s_samples - s_grid) % (120LL * SR);
            if (pos < 0) pos += 120LL * SR;
            s.cycle_pos = (float)pos / SR;
        }
        uint32_t b = __atomic_load_n(&s_lockseq, __ATOMIC_ACQUIRE);
        if (a != b) continue;
        if (st) *st = s;
        return n;
    }
    return 0;
}

/* ---- decode one candidate ---------------------------------------------------------------- */

#ifdef WSPR_DEBUG
#define DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...) ((void)0)
#endif

static bool decode_candidate(const peak_t *pk)
{
    /* refine: start ±1 frame, bin ±1, drift −3..+3 bins */
    float best = -2;
    int64_t bt = pk->start;
    int bb = pk->b0, bd = 0;
    for (int ds = -1; ds <= 1; ds++)
        for (int db = -1; db <= 1; db++)
            for (int dr = -3; dr <= 3; dr++) {
                float m = sync_metric(pk->start + ds, pk->b0 + db, dr);
                if (m > best) { best = m; bt = pk->start + ds; bb = pk->b0 + db; bd = dr; }
            }

    /* soft symbols: y > 0 means the "1" tone of the pair was stronger */
    float y[NSYM];
    double sumTone = 0;
    for (int i = 0; i < NSYM; i++) {
        int b = path_bin(bb, bd, i);
        if (b < 0 || b > NB - 7) return false;
        const float *r = row(bt + (int64_t)i * SYM_FR) + b;
        int s = SYNC[i];
        /* tone k sits in bin offset 2k; sync bit s selects the pair {tone s, tone 2+s} */
        float e0 = r[2 * s], e1 = r[2 * (2 + s)];
        y[i] = (e1 - e0) * 0.70710678f;
        sumTone += e0 > e1 ? e0 : e1;
    }

    /* metric per coded bit: log2(p(y|b)/p(y)) − R, Gaussian model, rate R = 1/2 */
    static float mt[NSYM][2];
    const float mu = WSPR_MU;
    for (int p = 0; p < NSYM; p++) {
        float yy = y[s_perm[p]];
        float z1 = -2.0f * mu * yy, z0 = 2.0f * mu * yy;
        mt[p][1] = 0.5f - log2f(1.0f + expf(z1 > 30 ? 30 : z1));
        mt[p][0] = 0.5f - log2f(1.0f + expf(z0 > 30 ? 30 : z0));
    }

    uint8_t bits[50];
    uint32_t cyc = 0;
    bool fok = fano_decode((const float (*)[2])mt, WSPR_DELTA, WSPR_MAXCYC, bits, &cyc);
    if (fok) s_fano_ok++;
    DBG("  cand start=%lld b0=%d (%.1f Hz) M=%.3f refined=%.3f drift=%d fano=%d cyc=%u\n", (long long)pk->start, bb, (B_LO + bb) * BIN_HZ, pk->metric, best, bd, fok, (unsigned)cyc);
    if (!fok) return false;

    /* re-encode and measure how well the soft symbols agree with the decoded word */
    uint32_t st = 0;
    double q = 0;
    for (int k = 0; k < 81; k++) {
        st = (st << 1) | (k < 50 ? bits[k] : 0u);
        int p1 = parity32(st & POLY1), p2 = parity32(st & POLY2);
        q += (p1 ? 1 : -1) * y[s_perm[2 * k]] + (p2 ? 1 : -1) * y[s_perm[2 * k + 1]];
    }
    q /= sqrt((double)NSYM);
    DBG("     q=%.2f\n", q);
    if (q < WSPR_QMIN) return false;

    char text[WSPR_TEXT_MAX];
    int rc = unpack_msg(bits, text, sizeof text);
    if (rc == 2) { s_other++; return true; }
    if (rc != 1) return false;

    float freq = (B_LO + bb) * BIN_HZ;
    float tone_pw = (float)(sumTone / NSYM);
    int snr = (int)lroundf(10.0f * log10f(tone_pw > 1.5f ? tone_pw - 1.0f : 0.5f) + WSPR_SNR_K);
    if (snr < -35) snr = -35;
    if (snr > 20) snr = 20;
    int64_t start_sample = bt * HOP;
    if (is_recent(text, freq, start_sample)) return true;
    add_message(text, freq, snr, (int)lroundf(bd * BIN_HZ), start_sample);
    return true;
}

static void drain(void);

/* Let other work in: consume audio that arrived meanwhile (the ring would overflow during a long
 * scan otherwise) and give the UI and idle tasks a slice. */
static void scan_slice(void)
{
    drain();
    WSPR_YIELD();
}

static void scan(void)
{
    int64_t k_new = s_frames - 1;
    int64_t s_max = k_new - SPAN_FR;
    if (s_max < 0) return;
    int64_t s_lo = s_next_start - SCAN_BACK;
    if (s_lo < 0) s_lo = 0;
    if (s_max - s_lo + 1 > SCAN_MAX) s_lo = s_max - SCAN_MAX + 1;
    if (s_lo < s_frames - FR + 1) s_lo = s_frames - FR + 1; /* rows still in the ring */
    if (s_lo > s_max) { s_next_start = s_max + 1; return; }
    int ns = (int)(s_max - s_lo + 1);
    s_in_scan = 1;
    s_scan_ref = s_frames;
    s_scans++;

    /* metric for every (start, lowest-tone bin) */
    static float acc[NB0], tot[NB0];
    for (int s = 0; s < ns; s++) {
        memset(acc, 0, sizeof acc);
        memset(tot, 0, sizeof tot);
        for (int i = 0; i < NSYM; i++) {
            const float *r = row(s_lo + s + (int64_t)i * SYM_FR);
            float w = SYNC[i] ? 1.0f : -1.0f;
            for (int b = 0; b < NB0; b++) {
                float ev = r[b] + r[b + 4], od = r[b + 2] + r[b + 6];
                acc[b] += w * (od - ev);
                tot[b] += ev + od;
            }
        }
        float *m = s_M + (size_t)s * NB0;
        for (int b = 0; b < NB0; b++) m[b] = tot[b] > 0 ? acc[b] / tot[b] : 0;
        if ((s & 7) == 7) scan_slice();
    }

    float best_all = 0; /* strongest sync correlation of this scan, even if below the threshold */
    for (int s = 0; s < ns; s++) {
        const float *m = s_M + (size_t)s * NB0;
        for (int b = 0; b < NB0; b++)
            if (m[b] > best_all) best_all = m[b];
    }
    s_best_m = best_all;

    /* pick the strongest peaks, suppressing their neighbourhood */
    peak_t pk[MAX_PEAKS];
    int np = 0;
    for (int n = 0; n < MAX_PEAKS; n++) {
        float best = WSPR_MTHR;
        int bs = -1, bbn = -1;
        for (int s = 0; s < ns; s++) {
            const float *m = s_M + (size_t)s * NB0;
            for (int b = 0; b < NB0; b++)
                if (m[b] > best) { best = m[b]; bs = s; bbn = b; }
        }
        if (bs < 0) break;
        pk[np].start = s_lo + bs;
        pk[np].b0 = bbn;
        pk[np].metric = best;
        np++;
        for (int s = bs - 6; s <= bs + 6; s++) {
            if (s < 0 || s >= ns) continue;
            float *m = s_M + (size_t)s * NB0;
            for (int b = bbn - 4; b <= bbn + 4; b++)
                if (b >= 0 && b < NB0) m[b] = -1;
        }
    }
    s_next_start = s_max + 1;

    for (int n = 0; n < np; n++) {
        s_cands++;
        decode_candidate(&pk[n]);
        scan_slice();
    }
    s_in_scan = 0;
}

/* ---- main processing ------------------------------------------------------------------------- */

static void soft_reset(bool keep_msgs)
{
    s_samples = 0;
    s_frames = 0;
    s_next_start = 0;
    s_next_scan_frame = SPAN_FR + SCAN_EVERY;
    s_grid_known = false;
    s_recent_n = 0;
    s_other = 0;
    s_in_scan = 0;
    if (s_hist) memset(s_hist, 0, WIN * sizeof(float));
    if (!keep_msgs) {
        s_scans = s_cands = s_fano_ok = 0;
        s_best_m = 0;
        s_drops = 0;
        s_st_state = 0;
        s_st_pct = 0;
        __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
        s_msg_total = 0;
        __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
    }
}

/* One hop (HOP audio samples) into the spectrogram; starts a sync scan when due. */
static void consume_hop(const float *hop)
{
    memmove(s_hist, s_hist + HOP, (WIN - HOP) * sizeof(float));
    memcpy(s_hist + WIN - HOP, hop, HOP * sizeof(float));
    s_samples += HOP;
    if (s_samples >= WIN) {
        make_frame();
        if (!s_in_scan && s_frames >= s_next_scan_frame) {
            s_next_scan_frame = s_frames + SCAN_EVERY;
            scan();
        }
    }
}

/* Consume all complete hops waiting in the ring. */
static void drain(void)
{
    static float hop[HOP];
    while ((uint32_t)(__atomic_load_n(&s_head, __ATOMIC_ACQUIRE) - s_tail) >= HOP) {
        /* while a scan runs, don't push the oldest rows (still being read) out of the frame ring */
        if (s_in_scan && s_frames >= s_scan_ref + 90) break;
        uint32_t t = s_tail;
        for (int i = 0; i < HOP; i++) hop[i] = s_ring[(t + i) & (RING_N - 1)];
        __atomic_store_n(&s_tail, t + HOP, __ATOMIC_RELEASE);
        consume_hop(hop);
    }
}

/* ---- self-test: synthetic WSPR audio through the real pipeline ---------------------------- */

static volatile int s_selftest_req;

void wspr_selftest(void) { s_selftest_req = 1; }

static float st_gauss(uint32_t *rng)
{
    float a = 0;
    for (int i = 0; i < 4; i++) {
        *rng ^= *rng << 13;
        *rng ^= *rng >> 17;
        *rng ^= *rng << 5;
        a += (float)(*rng >> 8) * (1.0f / 16777216.0f);
    }
    return (a - 2.0f) * 1.7320508f; /* ~N(0,1) */
}

/* Encoder for "K1ABC FN42 37" (per the protocol description), only used by the self-test. */
static void st_encode(uint8_t *tones)
{
    static const char call[6] = {' ', 'K', '1', 'A', 'B', 'C'};
    int c[6];
    for (int i = 0; i < 6; i++) {
        char ch = call[i];
        c[i] = (ch >= '0' && ch <= '9') ? ch - '0' : (ch == ' ' ? 36 : ch - 'A' + 10);
    }
    uint32_t n = (uint32_t)c[0];
    n = n * 36 + (uint32_t)c[1];
    n = n * 10 + (uint32_t)c[2];
    n = n * 27 + (uint32_t)(c[3] - 10);
    n = n * 27 + (uint32_t)(c[4] - 10);
    n = n * 27 + (uint32_t)(c[5] - 10);
    uint32_t m = ((uint32_t)((179 - 10 * ('F' - 'A') - 4) * 180 + 10 * ('N' - 'A') + 2) << 7) + 37u + 64u;
    uint8_t bits[81] = {0};
    for (int i = 0; i < 28; i++) bits[i] = (uint8_t)((n >> (27 - i)) & 1u);
    for (int i = 0; i < 22; i++) bits[28 + i] = (uint8_t)((m >> (21 - i)) & 1u);
    uint8_t par[NSYM];
    uint32_t reg = 0;
    for (int k = 0; k < 81; k++) {
        reg = (reg << 1) | bits[k];
        par[2 * k] = (uint8_t)parity32(reg & POLY1);
        par[2 * k + 1] = (uint8_t)parity32(reg & POLY2);
    }
    for (int p = 0; p < NSYM; p++) tones[s_perm[p]] = (uint8_t)(SYNC[s_perm[p]] + 2 * par[p]);
}

#define ST_SECONDS 250

static void selftest_run(void)
{
    uint8_t tones[NSYM];
    st_encode(tones);
    s_st_state = 1;
    s_st_pct = 0;
    soft_reset(true);
    uint32_t before = s_msg_total;
    uint32_t rng = 2463534242u;
    const float sigma = 1.0f, amp = 0.0913f; /* -20 dB in 2500 Hz: sqrt(2 * 0.01 * 2500 / 6000) */
    const float t0[2] = {3.7f, 123.7f};
    const int nhops = ST_SECONDS * SR / HOP;
    float hop[HOP], ph = 0;
    int64_t us0 = now_us();

    for (int h = 0; h < nhops; h++) {
        for (int i = 0; i < HOP; i++) {
            float t = (float)((int64_t)h * HOP + i) / SR, x = sigma * st_gauss(&rng);
            for (int k = 0; k < 2; k++) {
                float dt = t - t0[k];
                if (dt >= 0 && dt < NSYM * 0.68266667f) {
                    int sym = (int)(dt / 0.68266667f);
                    ph += 2.0f * (float)M_PI * (1500.0f + tones[sym] * 1.46484375f) / SR;
                    if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
                    x += amp * sinf(ph);
                }
            }
            hop[i] = x;
        }
        consume_hop(hop);
        if ((h & 15) == 15) {
            s_st_pct = h * 100 / nhops;
            s_tail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE); /* live audio is ignored */
            WSPR_YIELD();
        }
    }
    double secs = (double)(now_us() - us0) / 1e6;
    bool ok = false;
    for (uint32_t i = before; i < s_msg_total; i++)
        if (!strcmp(s_msgs[i % MSG_N].text, "K1ABC FN42 37")) ok = true;
    s_st_speed = (float)(ST_SECONDS / (secs > 0.001 ? secs : 0.001));
    s_st_pct = 100;
    s_st_state = ok ? 2 : 3;
    ESP_LOGI(TAG, "self-test %s: %d s of audio in %.1f s (%.1fx real time), %u spots", ok ? "PASSED" : "FAILED", ST_SECONDS,
             secs, (double)s_st_speed, (unsigned)(s_msg_total - before));
    s_tail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
    soft_reset(true); /* back to live audio with a fresh time base (the test spots stay listed) */
}

void wspr_process(void)
{
    init_once();
    if (!s_inited || !s_ring) return;
    if (s_reset_req) {
        s_reset_req = 0;
        s_tail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
        soft_reset(false);
    }
    if (s_selftest_req) {
        s_selftest_req = 0;
        selftest_run();
        return;
    }
    drain();
}

#ifdef ESP_PLATFORM
static void wspr_task(void *arg)
{
    (void)arg;
    while (1) {
        wspr_process();
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
#endif

void wspr_start(void)
{
    if (!s_ring) {
        s_ring = big_alloc(RING_N * sizeof(float));
        build_perm();
        if (!s_ring) {
            ESP_LOGE(TAG, "no memory for the audio ring");
            return;
        }
    }
#ifdef ESP_PLATFORM
    static int started;
    if (!started) {
        started = 1;
        xTaskCreatePinnedToCore(wspr_task, "wspr", 16 * 1024, NULL, 3, NULL, 0); /* below the UI task (6) */
    }
#endif
}
