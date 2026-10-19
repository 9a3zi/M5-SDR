/*
 * ft8.c — FT8 decoder glue. See ft8.h.
 *
 * Pipeline: 48 kHz audio → /4 → 12 kHz, slow level normalisation → ring buffer
 *   → (task) 0.16 s blocks → windowed 3840-point FFT, 2x time / 2x frequency
 *   oversampling → 8-bit log-magnitude waterfall (ft8_lib's format)
 *   → Costas-sync candidate search + LDPC/CRC decode (ft8_lib) → message list.
 *
 * Waterfall time is counted in absolute blocks since entering FT8 mode. A window
 * is "blocks [ws, ws+n)" of that stream, so any slot alignment can be decoded.
 */
#include "ft8.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "ft8/constants.h"
#include "ft8/decode.h"
#include "ft8/encode.h"
#include "ft8/message.h"
#include "fft/kiss_fftr.h"

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

static const char *TAG = "ft8";

/* ---- parameters ------------------------------------------------------------ */

#define SR          12000                 /* decoder sample rate                  */
#define BLOCK       1920                  /* samples per 0.16 s FT8 symbol        */
#define TIME_OSR    2
#define FREQ_OSR    2
#define SUBBLOCK    (BLOCK / TIME_OSR)
#define NFFT        (BLOCK * FREQ_OSR)    /* 3840                                 */
#define F_MIN_HZ    200
#define F_MAX_HZ    3000
#define MIN_BIN     ((int)(F_MIN_HZ * FT8_SYMBOL_PERIOD))
#define MAX_BIN     ((int)(F_MAX_HZ * FT8_SYMBOL_PERIOD) + 1)
#define NUM_BINS    (MAX_BIN - MIN_BIN)
#define STRIDE      (TIME_OSR * FREQ_OSR * NUM_BINS)

#define SLOT_SAMPLES (15 * SR)            /* 180000                               */
#define WIN_BLOCKS   92                   /* 14.7 s: decode window length         */
#define PRE_ROLL     3600                 /* locked window starts 0.3 s early     */
#define W_CAP        200                  /* waterfall blocks kept                */
#define W_DROP       90

#define MAX_CANDIDATES 140
#define MIN_SCORE      10
#define LDPC_ITER      25
#define MAX_DECODED    50

#define RING_N      65536                 /* 12 kHz samples (5.5 s)               */
#define MSG_N       24
#define SCAN_STEP   16                    /* blocks between scan attempts (2.56 s) */
#define EMPTY_UNLOCK 6                    /* empty slots before re-scanning        */

/* Calibration (from simulation, see host tests). */
#ifndef FT8_TIME_BIAS
#define FT8_TIME_BIAS 0.16f  /* seconds: reported time_sec minus true message start */
#endif
#ifndef FT8_SNR_K
#define FT8_SNR_K -26.0f       /* dB added to the tone/noise power ratio estimate     */
#endif

#define NORM_TARGET 0.03f                 /* normalised audio RMS (−30 dBFS)      */
#define NORM_ALPHA  (1.0f / (SR * 3.0f))  /* ~3 s level tracker                   */

static void copy_str(char *dst, size_t dst_size, const char *src)
{
    size_t n = strlen(src);
    if (n > dst_size - 1) n = dst_size - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/* ---- allocation ------------------------------------------------------------- */

static void *big_alloc(size_t n)
{
#ifdef ESP_PLATFORM
    void *p = heap_caps_calloc(1, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) return p;
#endif
    return calloc(1, n);
}

/* ---- producer side (DSP task) ----------------------------------------------- */

static float *s_ring;
static volatile uint32_t s_head, s_tail;
static volatile int s_reset_req;
static volatile int s_gap;

static float s_acc4;
static int s_n4;
static float s_pw = 0;
static uint32_t s_norm_n;

void ft8_feed(float y)
{
    s_acc4 += y;
    if (++s_n4 < 4) return;
    float x = s_acc4 * 0.25f;
    s_acc4 = 0;
    s_n4 = 0;
    if (!s_ring) return;

    /* slow level normalisation so the waterfall's fixed dB scale fits any gain */
    float a = (s_norm_n < (uint32_t)(SR * 3)) ? 1.0f / (float)(s_norm_n + 1) : NORM_ALPHA;
    s_norm_n++;
    s_pw += a * (x * x - s_pw);
    x *= NORM_TARGET / sqrtf(s_pw + 1e-12f);
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    uint32_t h = s_head;
    if (h - s_tail >= RING_N) { /* consumer too slow: samples lost → time base broken */
        s_gap = 1;
        return;
    }
    s_ring[h & (RING_N - 1)] = x;
    __atomic_store_n(&s_head, h + 1, __ATOMIC_RELEASE);
}

void ft8_reset(void)
{
    s_acc4 = 0;
    s_n4 = 0;
    s_pw = 0;
    s_norm_n = 0;
    s_reset_req = 1;
}

/* ---- consumer: waterfall ------------------------------------------------------ */

static int s_inited;
static uint8_t *W;                  /* [W_CAP][TIME_OSR][FREQ_OSR][NUM_BINS] */
static int64_t W_base;              /* absolute block index of W[0]          */
static int W_count;
static int64_t s_total_blocks;      /* blocks processed since reset          */

static float *s_window, *s_last_frame, *s_timedata;
static kiss_fft_cpx *s_freqdata;
static kiss_fftr_cfg s_fft_cfg;
static void *s_fft_work;

static float hann_i(int i, int N)
{
    float x = sinf((float)M_PI * i / N);
    return x * x;
}

static void init_once(void)
{
    if (s_inited) return;
    s_window = big_alloc(NFFT * sizeof(float));
    s_last_frame = big_alloc(NFFT * sizeof(float));
    s_timedata = big_alloc(NFFT * sizeof(float));
    s_freqdata = big_alloc((NFFT / 2 + 1) * sizeof(kiss_fft_cpx));
    W = big_alloc((size_t)W_CAP * STRIDE);
    const float fft_norm = 2.0f / NFFT;
    for (int i = 0; i < NFFT; i++) s_window[i] = fft_norm * hann_i(i, NFFT);
    size_t sz = 0;
    kiss_fftr_alloc(NFFT, 0, 0, &sz);
    s_fft_work = big_alloc(sz);
    s_fft_cfg = kiss_fftr_alloc(NFFT, 0, s_fft_work, &sz);
    s_inited = (W && s_window && s_last_frame && s_timedata && s_freqdata && s_fft_cfg) ? 1 : 0;
    if (!s_inited) ESP_LOGE(TAG, "out of memory");
}

/* One 0.16 s block → TIME_OSR waterfall rows appended to W. */
static void monitor_block(const float *frame)
{
    if (W_count >= W_CAP) { /* drop the oldest blocks, keep the newest */
        memmove(W, W + (size_t)W_DROP * STRIDE, (size_t)(W_CAP - W_DROP) * STRIDE);
        W_base += W_DROP;
        W_count -= W_DROP;
    }
    uint8_t *dst = W + (size_t)W_count * STRIDE;
    int fpos = 0, off = 0;
    for (int ts = 0; ts < TIME_OSR; ts++) {
        memmove(s_last_frame, s_last_frame + SUBBLOCK, (NFFT - SUBBLOCK) * sizeof(float));
        memcpy(s_last_frame + NFFT - SUBBLOCK, frame + fpos, SUBBLOCK * sizeof(float));
        fpos += SUBBLOCK;
        for (int i = 0; i < NFFT; i++) s_timedata[i] = s_window[i] * s_last_frame[i];
        kiss_fftr(s_fft_cfg, s_timedata, s_freqdata);
        for (int fs = 0; fs < FREQ_OSR; fs++) {
            for (int bin = MIN_BIN; bin < MAX_BIN; bin++) {
                int sb = bin * FREQ_OSR + fs;
                float m2 = s_freqdata[sb].r * s_freqdata[sb].r + s_freqdata[sb].i * s_freqdata[sb].i;
                float db = 10.0f * log10f(1e-12f + m2);
                int sc = (int)(2 * db + 240); /* 0..255 = −120..+7.5 dB, 0.5 dB steps */
                dst[off++] = (uint8_t)(sc < 0 ? 0 : (sc > 255 ? 255 : sc));
            }
        }
    }
    W_count++;
}

/* ---- callsign hash table (for <...> hashed calls), from ft8_lib's demo -------- */

#define HASH_N 256
static struct {
    char callsign[12];
    uint32_t hash; /* 8 MSBs: age; 22 LSBs: hash */
} s_hash[HASH_N];

static void hash_init(void) { memset(s_hash, 0, sizeof s_hash); }

static void hash_cleanup(uint8_t max_age)
{
    for (int i = 0; i < HASH_N; i++) {
        if (s_hash[i].callsign[0] == '\0') continue;
        uint8_t age = (uint8_t)(s_hash[i].hash >> 24);
        if (age > max_age) {
            s_hash[i].callsign[0] = '\0';
            s_hash[i].hash = 0;
        } else {
            s_hash[i].hash = (((uint32_t)age + 1u) << 24) | (s_hash[i].hash & 0x3FFFFFu);
        }
    }
}

static void hash_add(const char *callsign, uint32_t hash)
{
    uint16_t h10 = (hash >> 12) & 0x3FFu;
    int idx = (h10 * 23) % HASH_N;
    for (int n = 0; n < HASH_N && s_hash[idx].callsign[0] != '\0'; n++) {
        if (((s_hash[idx].hash & 0x3FFFFFu) == hash) && (0 == strcmp(s_hash[idx].callsign, callsign))) {
            s_hash[idx].hash &= 0x3FFFFFu; /* reset age */
            return;
        }
        idx = (idx + 1) % HASH_N;
    }
    if (s_hash[idx].callsign[0] != '\0') return; /* table full */
    copy_str(s_hash[idx].callsign, sizeof s_hash[idx].callsign, callsign);
    s_hash[idx].hash = hash;
}

static bool hash_lookup(ftx_callsign_hash_type_t type, uint32_t hash, char *callsign)
{
    uint8_t shift = (type == FTX_CALLSIGN_HASH_10_BITS) ? 12 : (type == FTX_CALLSIGN_HASH_12_BITS ? 10 : 0);
    uint16_t h10 = (hash >> (12 - shift)) & 0x3FFu;
    int idx = (h10 * 23) % HASH_N;
    for (int n = 0; n < HASH_N && s_hash[idx].callsign[0] != '\0'; n++) {
        if (((s_hash[idx].hash & 0x3FFFFFu) >> shift) == hash) {
            strcpy(callsign, s_hash[idx].callsign);
            return true;
        }
        idx = (idx + 1) % HASH_N;
    }
    callsign[0] = '\0';
    return false;
}

static ftx_callsign_hash_interface_t s_hash_if = {.lookup_hash = hash_lookup, .save_hash = hash_add};

/* ---- decoding one window --------------------------------------------------------- */

typedef struct {
    char text[FT8_TEXT_MAX];
    float freq;
    int snr;
    int64_t start; /* absolute sample index where the message starts */
} dec_t;

static dec_t s_dec[MAX_DECODED];

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

/* Approximate SNR (dB in 2500 Hz): mean power of the transmitted tone over the
 * message, relative to the noise floor measured in bins next to the signal
 * (median, so other stations don't bias it). */
static int estimate_snr(const ftx_waterfall_t *wf, const ftx_candidate_t *c, const uint8_t *tones)
{
    static float noise[FT8_NN * 8];
    int nn = 0, ns = 0;
    double ps = 0;
    for (int k = 0; k < FT8_NN; k++) {
        int blk = c->time_offset + k;
        if (blk < 0 || blk >= wf->num_blocks) continue;
        size_t base = (size_t)blk * wf->block_stride + (size_t)((c->time_sub * wf->freq_osr) + c->freq_sub) * wf->num_bins;
        int f0 = c->freq_offset;
        if (f0 + tones[k] < wf->num_bins) {
            ps += pow(10.0, WF_ELEM_MAG(wf->mag[base + f0 + tones[k]]) / 10.0);
            ns++;
        }
        for (int d = 0; d < 8; d++) { /* 4 bins below and 4 above the 8-tone block, with a 2-bin guard */
            int bin = (d < 4) ? f0 - 6 + d : f0 + 10 + (d - 4);
            if (bin < 0 || bin >= wf->num_bins) continue;
            noise[nn++] = (float)pow(10.0, WF_ELEM_MAG(wf->mag[base + bin]) / 10.0);
        }
    }
    if (ns < 10 || nn < 16) return -30;
    qsort(noise, nn, sizeof noise[0], cmp_f);
    double pn = noise[nn / 2] / 0.693; /* exponential noise: mean = median / ln 2 */
    double sig = ps / ns - pn;
    if (pn <= 0 || sig <= 0) return -30;
    double snr = 10.0 * log10(sig / pn) + FT8_SNR_K;
    if (snr < -30) snr = -30;
    if (snr > 30) snr = 30;
    return (int)lround(snr);
}

/* Decode blocks [ws, ws+nblk) of the absolute stream. Fills s_dec[], returns count. */
static int decode_window(int64_t ws, int nblk)
{
    int64_t rel = ws - W_base;
    if (rel < 0 || rel + nblk > W_count) return 0;

    ftx_waterfall_t wf = {
        .max_blocks = nblk,
        .num_blocks = nblk,
        .num_bins = NUM_BINS,
        .time_osr = TIME_OSR,
        .freq_osr = FREQ_OSR,
        .mag = W + (size_t)rel * STRIDE,
        .block_stride = STRIDE,
        .protocol = FTX_PROTOCOL_FT8,
    };

    static ftx_candidate_t cands[MAX_CANDIDATES];
    int ncand = ftx_find_candidates(&wf, MAX_CANDIDATES, cands, MIN_SCORE);

    static ftx_message_t decoded[MAX_DECODED];
    static ftx_message_t *dtab[MAX_DECODED];
    memset(dtab, 0, sizeof dtab);
    int ndec = 0;

    for (int i = 0; i < ncand && ndec < MAX_DECODED; i++) {
        const ftx_candidate_t *c = &cands[i];
        ftx_message_t msg;
        ftx_decode_status_t st;
        if (!ftx_decode_candidate(&wf, c, LDPC_ITER, &msg, &st)) continue;

        /* duplicate check (same payload found from two candidates) */
        int h = msg.hash % MAX_DECODED;
        bool dup = false, empty = false;
        for (int n = 0; n < MAX_DECODED && !empty && !dup; n++) {
            if (dtab[h] == NULL) empty = true;
            else if (dtab[h]->hash == msg.hash && !memcmp(dtab[h]->payload, msg.payload, sizeof msg.payload)) dup = true;
            else h = (h + 1) % MAX_DECODED;
        }
        if (!empty) continue;
        memcpy(&decoded[ndec], &msg, sizeof msg);
        dtab[h] = &decoded[ndec];

        char text[FTX_MAX_MESSAGE_LENGTH];
        ftx_message_offsets_t offs;
        if (ftx_message_decode(&msg, &s_hash_if, text, &offs) != FTX_MESSAGE_RC_OK) continue;

        uint8_t tones[FT8_NN];
        ft8_encode(msg.payload, tones);

        dec_t *d = &s_dec[ndec];
        memset(d, 0, sizeof *d);
        copy_str(d->text, sizeof d->text, text);
        d->freq = (MIN_BIN + c->freq_offset + (float)c->freq_sub / FREQ_OSR) / FT8_SYMBOL_PERIOD;
        float time_sec = (c->time_offset + (float)c->time_sub / TIME_OSR) * FT8_SYMBOL_PERIOD;
        d->start = ws * BLOCK + (int64_t)lroundf((time_sec - FT8_TIME_BIAS) * SR);
        d->snr = estimate_snr(&wf, c, tones);
        ndec++;
    }
    return ndec;
}

/* ---- slot-sync state ------------------------------------------------------------------ */

static bool s_locked;
static int64_t s_next_B;     /* absolute sample index of the next slot boundary to decode */
static uint16_t s_slot_no;
static int s_empty;
static int64_t s_next_scan;  /* block index at which to try the next scan decode */

/* ---- message list (UI) ------------------------------------------------------------- */

static ft8_msg_t s_msgs[MSG_N];
static uint32_t s_msg_total; /* messages ever added; next index = total % MSG_N */
static volatile uint32_t s_lockseq;
static ft8_status_t s_stat;

static struct {
    char text[FT8_TEXT_MAX];
    float freq;
    int64_t t;
} s_recent[MSG_N];
static int s_recent_n;

static bool is_recent(const dec_t *d)
{
    for (int i = 0; i < MSG_N && i < s_recent_n; i++) {
        if (!strcmp(s_recent[i].text, d->text) && fabsf(s_recent[i].freq - d->freq) < 15.0f &&
            llabs(s_recent[i].t - d->start) < 13 * SR)
            return true;
    }
    return false;
}

static void add_message(const dec_t *d, int64_t boundary, uint16_t slot)
{
    int i = s_recent_n % MSG_N;
    copy_str(s_recent[i].text, sizeof s_recent[i].text, d->text);
    s_recent[i].freq = d->freq;
    s_recent[i].t = d->start;
    s_recent_n++;

    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
    ft8_msg_t *m = &s_msgs[s_msg_total % MSG_N];
    m->seq = s_msg_total + 1;
    m->slot = slot;
    m->snr = (int16_t)d->snr;
    m->dt = (float)(d->start - boundary) / SR - 0.5f;
    m->freq = (int16_t)lroundf(d->freq);
    copy_str(m->text, sizeof m->text, d->text);
    s_msg_total++;
    s_stat.total = s_msg_total;
    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
}

int ft8_get_messages(ft8_msg_t *out, int max, ft8_status_t *st)
{
    for (int tries = 0; tries < 4; tries++) {
        uint32_t a = __atomic_load_n(&s_lockseq, __ATOMIC_ACQUIRE);
        if (a & 1) continue;
        uint32_t total = s_msg_total;
        int n = (int)(total < MSG_N ? total : MSG_N);
        if (n > max) n = max;
        for (int i = 0; i < n; i++) out[i] = s_msgs[(total - n + i) % MSG_N];
        ft8_status_t s = s_stat;
        uint32_t b = __atomic_load_n(&s_lockseq, __ATOMIC_ACQUIRE);
        if (a != b) continue;
        if (st) {
            *st = s;
            if (s.locked) {
                int64_t pos = (s_total_blocks * (int64_t)BLOCK - s_next_B) % SLOT_SAMPLES;
                if (pos < 0) pos += SLOT_SAMPLES;
                st->slot_pos = (float)pos / SR;
            }
        }
        return n;
    }
    return 0;
}

/* ---- slot synchronisation + scheduling ----------------------------------------------- */

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static int64_t floor_div(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
    return q;
}

static void soft_reset(void)
{
    W_base = 0;
    W_count = 0;
    s_total_blocks = 0;
    memset(s_last_frame, 0, NFFT * sizeof(float));
    s_locked = false;
    s_empty = 0;
    s_next_scan = 0;
    s_next_B = 0;
    s_slot_no = 0;
    s_recent_n = 0;
    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
    s_stat.locked = false;
    s_stat.slots = 0;
    s_stat.total = 0;
    s_stat.last_count = 0;
    s_msg_total = 0;
    __atomic_store_n(&s_lockseq, s_lockseq + 1, __ATOMIC_RELEASE);
}

static void lock_to(int64_t boundary)
{
    /* advance to the first boundary whose decode window is still in the future */
    while (floor_div(boundary - PRE_ROLL, BLOCK) + WIN_BLOCKS <= s_total_blocks) boundary += SLOT_SAMPLES;
    s_next_B = boundary;
    s_locked = true;
    s_empty = 0;
    s_stat.locked = true;
    ESP_LOGI(TAG, "slot phase locked");
}

static void do_scan_decode(void)
{
    if (s_total_blocks - W_base < WIN_BLOCKS + 1) return;
    int64_t ws = s_total_blocks - WIN_BLOCKS;
    int n = decode_window(ws, WIN_BLOCKS);
    s_stat.last_count = n;
    if (n <= 0) return;

    /* slot boundary estimate from the decoded messages (nominal start = boundary + 0.5 s) */
    int64_t b[MAX_DECODED];
    for (int i = 0; i < n; i++) b[i] = s_dec[i].start - SR / 2;
    qsort(b, n, sizeof b[0], cmp_i64);
    int64_t est = b[n / 2];

    for (int i = 0; i < n; i++)
        if (!is_recent(&s_dec[i])) add_message(&s_dec[i], est, s_slot_no);
    s_slot_no++;
    lock_to(est + SLOT_SAMPLES);
    s_stat.slots = 0;
}

static void do_locked_decode(void)
{
    int64_t ws = floor_div(s_next_B - PRE_ROLL, BLOCK);
    if (s_total_blocks < ws + WIN_BLOCKS) return; /* not enough audio yet */
    if (ws < W_base) { /* fell too far behind: skip this slot */
        s_next_B += SLOT_SAMPLES;
        return;
    }
    int n = decode_window(ws, WIN_BLOCKS);
    s_stat.last_count = n;
    int64_t boundary = s_next_B;
    int added = 0;
    for (int i = 0; i < n; i++) {
        if (!is_recent(&s_dec[i])) {
            add_message(&s_dec[i], boundary, s_slot_no);
            added++;
        }
    }
    hash_cleanup(10);
    s_slot_no++;
    s_stat.slots++;
    s_next_B += SLOT_SAMPLES;
    s_empty = (n > 0) ? 0 : s_empty + 1;
    if (s_empty >= EMPTY_UNLOCK) { /* nothing for a while: the phase may be wrong → scan again */
        s_locked = false;
        s_stat.locked = false;
        s_next_scan = s_total_blocks + SCAN_STEP;
        ESP_LOGI(TAG, "no decodes for %d slots: scanning again", s_empty);
    }
    (void)added;
}

static void schedule(void)
{
    if (s_locked) {
        do_locked_decode();
    } else if (s_total_blocks >= s_next_scan) {
        s_next_scan = s_total_blocks + SCAN_STEP;
        do_scan_decode();
    }
}

void ft8_process(void)
{
    init_once();
    if (!s_inited || !s_ring) return;
    if (s_reset_req) {
        s_reset_req = 0;
        s_tail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE); /* drop stale audio */
        s_gap = 0;
        soft_reset();
        hash_init();
    }
    static float frame[BLOCK];
    while ((uint32_t)(__atomic_load_n(&s_head, __ATOMIC_ACQUIRE) - s_tail) >= BLOCK) {
        if (s_gap) { /* samples were lost: the time base is no longer valid */
            s_gap = 0;
            s_tail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
            soft_reset();
            return;
        }
        uint32_t t = s_tail;
        for (int i = 0; i < BLOCK; i++) frame[i] = s_ring[(t + i) & (RING_N - 1)];
        __atomic_store_n(&s_tail, t + BLOCK, __ATOMIC_RELEASE);
        monitor_block(frame);
        s_total_blocks++;
        schedule();
    }
}

/* ---- task -------------------------------------------------------------------------- */

#ifdef ESP_PLATFORM
static void ft8_task(void *arg)
{
    (void)arg;
    while (1) {
        ft8_process();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

void ft8_start(void)
{
    if (!s_ring) {
        s_ring = big_alloc(RING_N * sizeof(float));
        hash_init();
        if (!s_ring) {
            ESP_LOGE(TAG, "no memory for the audio ring");
            return;
        }
    }
#ifdef ESP_PLATFORM
    static int started;
    if (!started) {
        started = 1;
        xTaskCreatePinnedToCore(ft8_task, "ft8", 24 * 1024, NULL, 2, NULL, 0);
    }
#endif
}
