/*
 * rs41.c — Vaisala RS41 radiosonde decoder. See rs41.h.
 *
 * Bit sync: the discriminator runs at 48 kHz = 10 samples/bit. Ten shift
 * registers (one per sample phase) hold the hard-sliced bits sampled at that
 * phase; when one equals the 64-bit frame header (either polarity) we lock,
 * pick the best phase from the next 10 samples and capture the frame at 1 bit
 * per 10 samples. The decision threshold is the mean of the last 640 samples
 * (one header length), which removes tuning offset.
 */
#include "rs41.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "rs41";

#define SPS          10   /* samples per bit @ 48 kHz / 4800 baud */
#define HDR_BITS     64
#define MEAN_WIN     (HDR_BITS * SPS)
#define HDR_MAX_ERR  4    /* tolerated header bit errors */

#define FRAME_STD 320
#define FRAME_EXT 518
#define MASK_LEN  64

/* Header as transmitted (first bit = MSB). */
static const uint64_t HDR = 0x086D53884469481FULL;
/* Same header as bytes (LSB-first on the air), before de-whitening. */
static const uint8_t HDR_BYTES[8] = {0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8};

/* Frame whitening mask (repeats every 64 bytes). */
static const uint8_t MASK[MASK_LEN] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98, 0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1, 0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C, 0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23, 0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1};

/* Block positions in the (de-whitened) frame. Each block: [type][len][data…][crc16 LE]. */
#define POS_FRAMETYPE 0x038 /* 0x0F = 320-byte frame, 0xF0 = 518-byte frame */
#define POS_STATUS    0x039 /* 0x79, len 0x28: frame number, serial          */
#define POS_GPS1      0x093 /* 0x7C, len 0x1E: GPS week, time of week        */
#define POS_GPS3      0x112 /* 0x7B, len 0x15: ECEF position + velocity      */

#define GPS_UTC_LEAP 18 /* GPS − UTC in seconds (unchanged since 2017) */

/* ---- CRC16-CCITT-FALSE (init 0xFFFF, poly 0x1021) --------------------------- */

static uint16_t crc16(const uint8_t *d, int len)
{
    uint16_t r = 0xFFFF;
    for (int i = 0; i < len; i++) {
        r ^= (uint16_t)d[i] << 8;
        for (int j = 0; j < 8; j++) r = (r & 0x8000) ? (uint16_t)((r << 1) ^ 0x1021) : (uint16_t)(r << 1);
    }
    return r;
}

/* ---- Reed-Solomon RS(255,231), GF(2^8) poly 0x11D, roots α^0..α^23 ----------- */

#define RS_N 255
#define RS_R 24
#define RS_T 12

static uint8_t gf_exp[512], gf_log[256];
static bool gf_ready;

static void gf_init(void)
{
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= 0x11D;
    }
    for (int i = 255; i < 512; i++) gf_exp[i] = gf_exp[i - 255];
    gf_ready = true;
}

static inline uint8_t gmul(uint8_t a, uint8_t b)
{
    return (a && b) ? gf_exp[gf_log[a] + gf_log[b]] : 0;
}
static inline uint8_t gdiv(uint8_t a, uint8_t b) /* b != 0 */
{
    return a ? gf_exp[gf_log[a] + 255 - gf_log[b]] : 0;
}

static int rs_syndromes(const uint8_t *cw, uint8_t *S)
{
    int any = 0;
    for (int i = 0; i < RS_R; i++) {
        uint8_t a = gf_exp[i], s = 0;
        for (int j = RS_N - 1; j >= 0; j--) s = (uint8_t)(gmul(s, a) ^ cw[j]);
        S[i] = s;
        if (s) any = 1;
    }
    return any;
}

/* cw[j] is the coefficient of x^j (parity first). Positions >= nvalid are the
 * zero-padding of the shortened code: an "error" there means mis-correction.
 * Returns number of corrected symbols, or -1 if uncorrectable. */
static int rs_decode(uint8_t *cw, int nvalid)
{
    uint8_t S[RS_R];
    if (!rs_syndromes(cw, S)) return 0;

    /* Berlekamp–Massey */
    uint8_t C[RS_R + 1] = {1}, B[RS_R + 1] = {1}, T[RS_R + 1];
    int L = 0, m = 1;
    uint8_t bb = 1;
    for (int n = 0; n < RS_R; n++) {
        uint8_t d = S[n];
        for (int i = 1; i <= L; i++) d ^= gmul(C[i], S[n - i]);
        if (!d) { m++; continue; }
        memcpy(T, C, sizeof T);
        uint8_t coef = gdiv(d, bb);
        for (int i = 0; i + m <= RS_R; i++) C[i + m] ^= gmul(coef, B[i]);
        if (2 * L <= n) {
            L = n + 1 - L;
            memcpy(B, T, sizeof B);
            bb = d;
            m = 1;
        } else {
            m++;
        }
    }
    if (L == 0 || L > RS_T) return -1;

    /* Chien search: roots of Λ at x = α^-j mark error positions j. */
    int pos[RS_T], ne = 0;
    for (int j = 0; j < RS_N; j++) {
        uint8_t x = gf_exp[(255 - j) % 255], v = 0;
        for (int i = L; i >= 0; i--) v = (uint8_t)(gmul(v, x) ^ C[i]);
        if (!v) {
            if (ne >= RS_T) return -1;
            pos[ne++] = j;
        }
    }
    if (ne != L) return -1;

    /* Ω(x) = S(x)Λ(x) mod x^R */
    uint8_t Om[RS_R];
    for (int i = 0; i < RS_R; i++) {
        uint8_t o = 0;
        for (int k = 0; k <= i && k <= L; k++) o ^= gmul(C[k], S[i - k]);
        Om[i] = o;
    }

    /* Forney (first root α^0): e = X · Ω(X⁻¹) / Λ'(X⁻¹) */
    for (int k = 0; k < ne; k++) {
        int j = pos[k];
        if (j >= nvalid) return -1;
        uint8_t X = gf_exp[j], xi = gf_exp[(255 - j) % 255];
        uint8_t num = 0, den = 0;
        for (int i = RS_R - 1; i >= 0; i--) num = (uint8_t)(gmul(num, xi) ^ Om[i]);
        for (int i = 1; i <= L; i += 2) { /* Λ' keeps odd terms: C[i]·x^(i-1) */
            uint8_t p = (i == 1) ? 1 : gf_exp[(gf_log[xi] * (i - 1)) % 255];
            if (xi == 1) p = 1;
            den ^= gmul(C[i], p);
        }
        if (!den) return -1;
        cw[j] ^= gmul(X, gdiv(num, den));
    }

    if (rs_syndromes(cw, S)) return -1; /* must now be a valid codeword */
    return ne;
}

/* The frame carries two interleaved codewords: parity at 8..55 (24 bytes each),
 * message bytes from 56 alternating between them. Returns symbols fixed, or -1. */
static int rs41_ecc(uint8_t *f, int len)
{
    int msglen = (len - 56) / 2;
    if (msglen <= 0 || msglen > RS_N - RS_R) return -1;
    uint8_t cw1[RS_N] = {0}, cw2[RS_N] = {0};
    for (int i = 0; i < RS_R; i++) {
        cw1[i] = f[8 + i];
        cw2[i] = f[8 + RS_R + i];
    }
    for (int i = 0; i < msglen; i++) {
        cw1[RS_R + i] = f[56 + 2 * i];
        cw2[RS_R + i] = f[56 + 2 * i + 1];
    }
    int e1 = rs_decode(cw1, RS_R + msglen);
    int e2 = rs_decode(cw2, RS_R + msglen);
    if (e1 < 0 || e2 < 0) return -1;
    for (int i = 0; i < RS_R; i++) {
        f[8 + i] = cw1[i];
        f[8 + RS_R + i] = cw2[i];
    }
    for (int i = 0; i < msglen; i++) {
        f[56 + 2 * i] = cw1[RS_R + i];
        f[56 + 2 * i + 1] = cw2[RS_R + i];
    }
    return e1 + e2;
}

/* ---- frame → telemetry ------------------------------------------------------ */

static void ecef2lla(const double X[3], double *lat, double *lon, double *alt)
{
    const double a = 6378137.0, b = 6356752.31424518;
    const double e2 = (a * a - b * b) / (a * a), ee2 = (a * a - b * b) / (b * b);
    double p = sqrt(X[0] * X[0] + X[1] * X[1]);
    double t = atan2(X[2] * a, p * b);
    double phi = atan2(X[2] + ee2 * b * pow(sin(t), 3), p - e2 * a * pow(cos(t), 3));
    double R = a / sqrt(1.0 - e2 * sin(phi) * sin(phi));
    *alt = p / cos(phi) - R;
    *lat = phi * 180.0 / M_PI;
    *lon = atan2(X[1], X[0]) * 180.0 / M_PI;
}

static inline int32_t le32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static inline int16_t le16s(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* Position block (0x7B, and 0x82 of newer sondes): ECEF X,Y,Z (int32, cm) at +0,
 * velocity (int16, cm/s) at +12. */
static bool parse_pos(const uint8_t *d, rs41_info_t *info)
{
    double X[3], V[3], lat, lon, alt;
    for (int k = 0; k < 3; k++) {
        X[k] = le32(d + 4 * k) / 100.0;
        V[k] = le16s(d + 12 + 2 * k) / 100.0;
    }
    if (X[0] == 0 && X[1] == 0 && X[2] == 0) return false; /* sondes sometimes send all-zero */
    ecef2lla(X, &lat, &lon, &alt);
    if (!(alt > -1000.0 && alt < 80000.0)) return false;
    double phi = lat * M_PI / 180.0, lam = lon * M_PI / 180.0;
    double vN = -V[0] * sin(phi) * cos(lam) - V[1] * sin(phi) * sin(lam) + V[2] * cos(phi);
    double vE = -V[0] * sin(lam) + V[1] * cos(lam);
    double vU = V[0] * cos(phi) * cos(lam) + V[1] * cos(phi) * sin(lam) + V[2] * sin(phi);
    double dir = atan2(vE, vN) * 180.0 / M_PI;
    if (dir < 0) dir += 360.0;
    info->lat = lat;
    info->lon = lon;
    info->alt = alt;
    info->vh = (float)sqrt(vN * vN + vE * vE);
    info->vd = (float)dir;
    info->vv = (float)vU;
    info->have_pos = true;
    return true;
}

static void set_time_of_day(rs41_info_t *info, int64_t gps_sec_of_day)
{
    int64_t sec = gps_sec_of_day - GPS_UTC_LEAP;
    if (sec < 0) sec += 86400;
    sec %= 86400;
    info->hh = (int)(sec / 3600);
    info->mm = (int)((sec % 3600) / 60);
    info->ss = (int)(sec % 60);
    info->have_time = true;
}

/* One block at f[p]: [type][len][data…][crc16 LE]. Parses it if the CRC is good.
 * Returns the type if valid, 0 otherwise. `want` (if non-zero) restricts the
 * accepted type (used by the fixed-position fallback). */
static int parse_block(const uint8_t *f, int p, int len, uint8_t want, rs41_info_t *info)
{
    if (p + 2 > len) return 0;
    uint8_t typ = f[p], bl = f[p + 1];
    if (want && typ != want) return 0;
    if (p + 2 + bl + 2 > len) return 0;
    uint16_t got = (uint16_t)(f[p + 2 + bl] | (f[p + 3 + bl] << 8));
    if (got != crc16(f + p + 2, bl)) return 0;
    const uint8_t *d = f + p + 2;

    switch (typ) {
    case 0x79: /* status: frame number, serial */
        if (bl < 10) break;
        info->frame_no = (uint16_t)(d[0] | (d[1] << 8));
        for (int i = 0; i < 8; i++) {
            uint8_t c = d[2 + i];
            info->id[i] = (c >= 0x20 && c < 0x7F) ? (char)c : '?';
        }
        info->id[8] = '\0';
        info->have_id = true;
        break;
    case 0x80: /* RS41-SGM: encrypted payload (position/time are not readable) */
        info->encrypted = true;
        break;
    case 0x7C: /* GPS week + time of week */
        if (bl < 6) break;
        info->week = (uint16_t)(d[0] | (d[1] << 8));
        info->tow_ms = (uint32_t)le32(d + 2);
        set_time_of_day(info, (int64_t)((info->tow_ms / 1000) % 86400));
        break;
    case 0x7B: /* position (classic) */
        if (bl < 19) break;
        if (parse_pos(d, info)) info->sats = d[18];
        break;
    case 0x82: /* position + date/time (newer sondes) */
        if (bl < 25) break;
        parse_pos(d, info);
        /* date/time of this block is UTC already: year(2) month day hour min sec at +18 */
        {
            int64_t sod = d[22] * 3600 + d[23] * 60 + d[24];
            if (d[22] < 24 && d[23] < 60 && d[24] < 61) {
                info->hh = (int)(sod / 3600);
                info->mm = (int)((sod % 3600) / 60);
                info->ss = (int)(sod % 60);
                info->have_time = true;
            }
        }
        break;
    case 0x83: /* satellite info (newer sondes): bitmask of used SVs at +18 */
        if (bl < 25) break;
        {
            int n = 0; /* 21 bytes = 168 SV flags (GPS, Galileo, …) at +4 */
            for (int i = 0; i < 21 * 8; i++) if (d[4 + i / 8] & (1 << (i & 7))) n++;
            info->sats = (uint8_t)n;
        }
        break;
    default:
        break; /* PTU/measurement, aux, padding …: CRC fine, nothing to extract */
    }
    return typ;
}

int rs41_decode_frame(const uint8_t *raw, int len, rs41_info_t *info)
{
    if (len != FRAME_STD && len != FRAME_EXT) return 0;
    if (!gf_ready) gf_init();

    uint8_t f[FRAME_EXT] = {0};
    for (int i = 0; i < len; i++) f[i] = raw[i] ^ MASK[i % MASK_LEN];

    /* How many fixed marker bytes survived the channel (before error correction)? */
    {
        int mk = 0;
        mk += (f[POS_FRAMETYPE] == 0x0F || f[POS_FRAMETYPE] == 0xF0);
        mk += (f[POS_STATUS] == 0x79) + (f[POS_STATUS + 1] == 0x28);
        mk += (f[POS_GPS1] == 0x7C) + (f[POS_GPS1 + 1] == 0x1E);
        mk += (f[POS_GPS3] == 0x7B) + (f[POS_GPS3 + 1] == 0x15);
        info->mk = (uint8_t)mk;
    }

    info->rs_fixed = rs41_ecc(f, len); /* on failure f is untouched; CRCs still decide */

    /* Walk the block chain from 0x39 (each block carries its own length). */
    int ok = 0;
    uint8_t seen[256 / 8] = {0};
    info->nblk = 0;
    for (int p = POS_STATUS; p + 4 <= len;) {
        uint8_t typ = f[p];
        int bl = f[p + 1];
        int t = parse_block(f, p, len, 0, info);
        if (t) {
            ok++;
            seen[t >> 3] |= (uint8_t)(1 << (t & 7));
            if (info->nblk < sizeof info->btype) info->btype[info->nblk++] = (uint8_t)t;
        }
        if (typ == 0) break;
        p += bl + 4;
    }
    /* Fallback if the chain broke (e.g. one damaged length byte): known fixed positions. */
    static const struct { int pos; uint8_t type; } FIX[] = {
        {POS_STATUS, 0x79}, {POS_GPS1, 0x7C}, {POS_GPS3, 0x7B}};
    for (unsigned i = 0; i < sizeof FIX / sizeof FIX[0]; i++) {
        if (seen[FIX[i].type >> 3] & (1 << (FIX[i].type & 7))) continue;
        if (parse_block(f, FIX[i].pos, len, FIX[i].type, info)) {
            ok++;
            if (info->nblk < sizeof info->btype) info->btype[info->nblk++] = FIX[i].type;
        }
    }
    info->blk_ok = (uint8_t)ok;

    if (ok) info->frames_ok++;
    else info->frames_bad++;
    info->seq++;
    return ok;
}

/* ---- shared telemetry (DSP task writes, UI task reads; seqlock) ------------- */

static rs41_info_t s_info;              /* written only by the DSP task */
static rs41_info_t s_pub;               /* published copy */
static volatile uint32_t s_lock;        /* odd while s_pub is being written */

static void publish(void)
{
    __atomic_store_n(&s_lock, s_lock + 1, __ATOMIC_RELEASE);
    s_pub = s_info;
    __atomic_store_n(&s_lock, s_lock + 1, __ATOMIC_RELEASE);
}

bool rs41_get_info(rs41_info_t *out)
{
    for (int tries = 0; tries < 4; tries++) {
        uint32_t a = __atomic_load_n(&s_lock, __ATOMIC_ACQUIRE);
        if (a & 1) continue;
        *out = s_pub;
        uint32_t b = __atomic_load_n(&s_lock, __ATOMIC_ACQUIRE);
        if (a == b) return out->seq != 0;
    }
    return false;
}

/* ---- bit sync + frame capture ---------------------------------------------- */

enum { ST_SEARCH, ST_SETTLE, ST_CAPTURE };

static int s_state;
static uint32_t s_n; /* free-running sample counter */

#ifndef RS41_MA_LEN
#define RS41_MA_LEN 8 /* discriminator smoothing (samples @48 kHz) */
#endif
static float s_ma[RS41_MA_LEN];
static int s_ma_i;
static float s_ma_sum;

/* last 8 smoothed samples (timing tracker looks 3 samples either side of the bit centre) */
static float s_hist[8];
static int s_hi;
static float s_tacc; /* leaky early/late accumulator */
#define BACK(k) s_hist[(s_hi - 1 - (k)) & 7]
#ifndef T_LIMIT
#define T_LIMIT 1.0f
#endif
#ifndef T_LEAK
#define T_LEAK  0.94f
#endif

static float s_ring[MEAN_WIN];
static int s_ring_i;
static float s_ring_sum;

static uint64_t s_sr[SPS];

static int s_p0, s_settle_k, s_pol;
static int s_dist[SPS];
static float s_thr, s_eye;

static int s_wait;
static uint8_t s_raw[FRAME_EXT];
static int s_nbytes, s_len, s_bitcnt;
static uint8_t s_cur;

void rs41_reset(void)
{
    s_state = ST_SEARCH;
    s_n = 0;
    memset(s_ma, 0, sizeof s_ma);
    s_ma_sum = 0;
    memset(s_ring, 0, sizeof s_ring);
    s_ma_i = s_ring_i = 0;
    s_hi = 0;
    memset(s_hist, 0, sizeof s_hist);
    s_tacc = 0;
    s_ring_sum = 0;
    memset(s_sr, 0, sizeof s_sr);
    if (!gf_ready) gf_init();
}

static void finish_frame(void)
{
    int ok = rs41_decode_frame(s_raw, s_len, &s_info);
    s_info.eye = fabsf(s_eye);
    char bt[40] = "";
    for (int i = 0; i < s_info.nblk && i < 10; i++) {
        char t[5];
        snprintf(t, sizeof t, "%02X ", s_info.btype[i]);
        strcat(bt, t);
    }
    if (!ok) {
        ESP_LOGW(TAG, "bad frame: markers %d/7, RS %d, eye %.2f rad/sample (good ~0.6)", s_info.mk, s_info.rs_fixed,
                 s_info.eye);
        /* Raw de-whitened bytes of the first few bad frames (then every 20th), so the
         * frame layout of an unsupported sonde can be inspected offline. */
        if (s_info.frames_bad <= 3 || s_info.frames_bad % 20 == 0) {
            char line[3 * 32 + 1];
            for (int off = 0; off < 0xC0 && off < s_len; off += 32) {
                int n = 0;
                for (int i = 0; i < 32 && off + i < s_len; i++)
                    n += snprintf(line + n, sizeof line - n, "%02X ", s_raw[off + i] ^ MASK[(off + i) % MASK_LEN]);
                ESP_LOGW(TAG, "raw[%03X] %s", off, line);
            }
        }
    } else {
        if (s_info.seq <= 3) ESP_LOGI(TAG, "valid blocks: %s", bt);
        ESP_LOGI(TAG, "%s #%u  %.5f %.5f  %.0f m  sats %u  RS %d", s_info.id, (unsigned)s_info.frame_no,
                 s_info.lat, s_info.lon, s_info.alt, (unsigned)s_info.sats, s_info.rs_fixed);
    }
    publish();
    s_state = ST_SEARCH;
    memset(s_sr, 0, sizeof s_sr);
}

static inline int hdr_dist(uint64_t sr, int pol)
{
    int d = __builtin_popcountll(sr ^ HDR);
    return pol ? HDR_BITS - d : d;
}

void rs41_feed(float d)
{
    /* 4-sample moving average of the discriminator output */
    s_ma_sum += d - s_ma[s_ma_i];
    s_ma[s_ma_i] = d;
    if (++s_ma_i >= RS41_MA_LEN) s_ma_i = 0;
    float s = s_ma_sum * (1.0f / RS41_MA_LEN);

    /* running mean over one header length = decision threshold */
    s_ring_sum += s - s_ring[s_ring_i];
    s_ring[s_ring_i] = s;
    if (++s_ring_i >= MEAN_WIN) s_ring_i = 0;
    float thr = s_ring_sum * (1.0f / MEAN_WIN);
    s_hist[s_hi] = s;
    s_hi = (s_hi + 1) & 7;

    int p = (int)(s_n % SPS);
    s_n++;

    switch (s_state) {
    case ST_SEARCH: {
        s_sr[p] = (s_sr[p] << 1) | (s > thr ? 1u : 0u);
        int dn = __builtin_popcountll(s_sr[p] ^ HDR);
        if (dn <= HDR_MAX_ERR || HDR_BITS - dn <= HDR_MAX_ERR) {
            s_pol = (dn > HDR_BITS / 2);
            s_p0 = p;
            s_settle_k = 0;
            s_dist[0] = hdr_dist(s_sr[p], s_pol);
            s_state = ST_SETTLE;
        }
        break;
    }
    case ST_SETTLE: {
        /* the next 9 samples show how well the neighbouring phases match */
        s_sr[p] = (s_sr[p] << 1) | (s > thr ? 1u : 0u);
        s_settle_k++;
        s_dist[s_settle_k] = hdr_dist(s_sr[p], s_pol);
        if (s_settle_k >= SPS - 1) {
            int best = 99;
            for (int k = 0; k < SPS; k++) if (s_dist[k] < best) best = s_dist[k];
            int lo = -1, hi = -1;
            for (int k = 0; k < SPS; k++) {
                if (s_dist[k] <= best + 1) { if (lo < 0) lo = k; hi = k; }
            }
            int j = (lo + hi) / 2; /* centre of the matching run */
            s_wait = j + 1 + 3;    /* first frame-bit centre is j+1 samples away; we act 3 samples later */
            s_tacc = 0;
            /* Decision threshold from the header itself: midway between the mean
             * level of its "high" and "low" bits at the chosen sampling phase.
             * (The plain window mean is biased: the header is only 39 % ones.) */
            {
                float hi = 0, lo = 0;
                int nh = 0, nl = 0;
                for (int k = 0; k < HDR_BITS; k++) {
                    int back = (SPS - 1 - j) + SPS * k;
                    int idx = s_ring_i - 1 - back;
                    if (idx < 0) idx += MEAN_WIN;
                    if (((int)((HDR >> k) & 1u)) ^ s_pol) { hi += s_ring[idx]; nh++; }
                    else { lo += s_ring[idx]; nl++; }
                }
                if (nh && nl) {
                    hi /= nh; lo /= nl;
                    s_thr = 0.5f * (hi + lo);
                    s_eye = hi - lo;
                } else {
                    s_thr = thr;
                    s_eye = 0;
                }
            }
            memcpy(s_raw, HDR_BYTES, 8);
            s_nbytes = 8;
            s_bitcnt = 0;
            s_cur = 0;
            s_len = FRAME_STD;
            s_state = ST_CAPTURE;
        }
        break;
    }
    case ST_CAPTURE:
        if (--s_wait > 0) break;
        s_wait = SPS;
        {
            /* bit centre = 3 samples ago; early/late samples 3 either side */
            float c = BACK(3), e = BACK(6), l = BACK(0);
            int bit = (c > s_thr) ? 1 : 0;
            if (s_pol) bit ^= 1;

            /* Timing tracker: near a bit edge the eye closes. If we sample late, the
             * late sample is closer to threshold than the early one (and vice versa). */
            float sw = s_eye < 0 ? -s_eye : s_eye;
            if (sw < 1e-4f) sw = 1e-4f;
            float me = fabsf(e - s_thr), ml = fabsf(l - s_thr);
            s_tacc = s_tacc * T_LEAK + (me - ml) / sw;
            if (s_tacc > T_LIMIT) { s_wait = SPS - 1; s_tacc = 0; }       /* too late: sample earlier */
            else if (s_tacc < -T_LIMIT) { s_wait = SPS + 1; s_tacc = 0; } /* too early: sample later */

            s_cur |= (uint8_t)(bit << s_bitcnt); /* LSB first */
            if (++s_bitcnt == 8) {
                s_raw[s_nbytes++] = s_cur;
                s_cur = 0;
                s_bitcnt = 0;
                if (s_nbytes == POS_FRAMETYPE + 1) {
                    uint8_t t = s_raw[POS_FRAMETYPE] ^ MASK[POS_FRAMETYPE % MASK_LEN];
                    s_len = (t == 0xF0) ? FRAME_EXT : FRAME_STD;
                }
                if (s_nbytes >= s_len) finish_frame();
            }
        }
        break;
    }
}
