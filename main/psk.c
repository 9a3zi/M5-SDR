/*
 * psk.c — PSK Reporter upload, see psk.h.
 *
 * Datagram format: the "IPFIX" layout from pskreporter.info/pskdev.html,
 * the same one WSJT-X and the reference pskr.pl script send by UDP to
 * report.pskreporter.info:4739. A datagram carries a receiver template+record
 * (our callsign/locator/software) and a sender template+records, one per spot.
 */
#include "psk.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs.h"

#include "net_cfg.h"
#include "station_settings.h"
#include "wifi_net.h"

static const char *TAG = "psk";

#define SPOT_CALL_MAX 14
#define SPOT_GRID_MAX 6
#define SPOT_MODE_MAX 6
#define QUEUE_N       64
#define DEDUP_N       32
#define DEDUP_AGE_S   300   /* same call+mode+band more often than this → drop */
#define BAND_HZ       10000000u /* dedup band bucket, as in pskr.pl */
#define MAX_DGRAM     1200  /* stay well under one Ethernet MTU */
#define STR_(x) #x
#define STR(x) STR_(x)

typedef struct {
    char call[SPOT_CALL_MAX + 1];
    char grid[SPOT_GRID_MAX + 1];
    char mode[SPOT_MODE_MAX + 1];
    uint32_t freq; /* Hz */
    int8_t snr;
    uint32_t t;    /* unix seconds of the decode */
} spot_t;

static spot_t s_q[QUEUE_N];
static int s_qh, s_qn;
static SemaphoreHandle_t s_mu;

/* recent-upload cache for dedup: "MODE call band" + time */
static uint32_t s_ded_t[DEDUP_N];
static char s_ded_key[DEDUP_N][32];
static int s_ded_i;

static volatile bool s_enabled;
static uint32_t s_sent, s_seq, s_rid;

/* ---- protocol templates (pskdev.html) -------------------------------------- */

/* Receiver template, set 3, id 0x9992 — official 4-field option
 * (receiverCallsign, receiverLocator, decodingSoftware, rigInformation),
 * byte-exact per pskdev.html (note the trailing 00 01 before fields). */
static const uint8_t TPL_RX[44] = {
    0x00, 0x03, 0x00, 0x2C, 0x99, 0x92, 0x00, 0x04, 0x00, 0x01,
    0x80, 0x02, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* receiverCallsign   */
    0x80, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* receiverLocator    */
    0x80, 0x08, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* decodingSoftware   */
    0x80, 0x0D, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* rigInformation     */
    0x00, 0x00
};
/* Sender template — the OFFICIAL 8-field option from pskdev.html
 * (senderCallsign, frequency, sNR, iMD, mode, informationSource, senderLocator,
 * flowStartSeconds). Note: a "sNR without iMD, with locator" template is NOT
 * among the official options, so iMD is included (0 = not measured). */
/* 8 (set hdr) + 7×8 (enterprise fields) + 4 (flowStartSeconds) = 68 = 0x44.
 * NOTE: the array size must match the 0x44 length byte exactly — a larger
 * size would zero-fill and inject phantom bytes into every datagram! */
static const uint8_t TPL_TX[68] = {
    0x00, 0x02, 0x00, 0x44, 0x99, 0x93, 0x00, 0x08,
    0x80, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* senderCallsign    */
    0x80, 0x05, 0x00, 0x04, 0x00, 0x00, 0x76, 0x8F, /* frequency, u32 Hz */
    0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F, /* sNR, int8         */
    0x80, 0x07, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F, /* iMD, int8         */
    0x80, 0x0A, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* mode, string      */
    0x80, 0x0B, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F, /* informationSource */
    0x80, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, /* senderLocator     */
    0x00, 0x96, 0x00, 0x04                          /* flowStartSeconds  */
};
_Static_assert(sizeof(TPL_TX) == 0x44, "TPL_TX must be exactly 68 bytes");
_Static_assert(sizeof(TPL_RX) == 0x2C, "TPL_RX must be exactly 44 bytes");

/* ---- little writers --------------------------------------------------------- */

static uint8_t *put16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)v; return p; }
static uint8_t *put32(uint8_t *p, uint32_t v)
{
    *p++ = (uint8_t)(v >> 24); *p++ = (uint8_t)(v >> 16);
    *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)v;
    return p;
}

/* append a "<len><bytes>" variable field */
static uint8_t *put_str(uint8_t *p, const char *s)
{
    size_t n = strlen(s);
    if (n > 255) n = 255;
    *p++ = (uint8_t)n;
    memcpy(p, s, n);
    return p + n;
}

/* ---- message parsing ---------------------------------------------------------- */

static bool valid_call(const char *s)
{
    size_t n = strlen(s);
    if (n < 3 || n > SPOT_CALL_MAX) return false;
    bool digit = false;
    for (const char *p = s; *p; p++) {
        char c = (char)toupper((unsigned char)*p);
        if (c >= '0' && c <= '9') digit = true;
        else if (!(c >= 'A' && c <= 'Z') && c != '/') return false;
    }
    return digit;
}

static bool valid_grid(const char *s)
{
    if (strlen(s) != 4) return false;
    return s[0] >= 'A' && s[0] <= 'R' && s[1] >= 'A' && s[1] <= 'R' &&
           isdigit((unsigned char)s[2]) && isdigit((unsigned char)s[3]);
}

static int split(const char *text, char tok[][16], int max_tok)
{
    int n = 0;
    const char *p = text;
    while (*p && n < max_tok) {
        while (*p == ' ') p++;
        if (!*p) break;
        int k = 0;
        while (*p && *p != ' ' && k < 15) tok[n][k++] = *p++;
        tok[n][k] = '\0';
        n++;
    }
    return n;
}

/* Extract sender callsign + (optional) 4-char locator from a decoded message.
 * Mirrors pskr.pl: for "CQ …" the sender is after CQ (skip 2-char directives
 * like "CQ DX9XYZ …" / "CQ NA …"), otherwise the first token. */
static int parse_msg(const char *text, const char *mode, char *call, size_t cn, char *grid, size_t gn)
{
    char tok[8][16];
    int n = split(text, tok, 8);
    if (n == 0) return 0;
    call[0] = grid[0] = '\0';

    int ci = 0;
    if (!strcmp(tok[0], "CQ")) {
        ci = 1;
        if (n > 2 && strlen(tok[1]) <= 2) ci = 2; /* "CQ DX9…"→1, "CQ NA S52…"→2 */
    } else if (!strcmp(tok[0], "QRZ")) {
        ci = 1;
    }
    if (ci >= n) return 0;
    if (!valid_call(tok[ci])) return 0;
    snprintf(call, cn, "%s", tok[ci]);

    /* locator: any 4-char grid token (not the callsign slot) */
    for (int i = 0; i < n; i++) {
        if (i == ci) continue;
        if (valid_grid(tok[i])) { snprintf(grid, gn, "%s", tok[i]); break; }
    }
    return 1;
}

/* ---- queue --------------------------------------------------------------------- */

static bool dedup_check_and_add(const char *mode, const char *call, uint32_t freq, uint32_t now)
{
    char key[32];
    snprintf(key, sizeof key, "%s %s %u", mode, call, (unsigned)(freq / BAND_HZ));
    for (int i = 0; i < DEDUP_N; i++) {
        if (s_ded_t[i] && now - s_ded_t[i] < DEDUP_AGE_S && !strcmp(s_ded_key[i], key)) return false;
    }
    s_ded_i = (s_ded_i + 1) % DEDUP_N;
    s_ded_t[s_ded_i] = now;
    snprintf(s_ded_key[s_ded_i], sizeof s_ded_key[0], "%s", key);
    return true;
}

void psk_spot_msg(const char *text, double freq_hz, int snr_db, const char *mode)
{
    if (!s_enabled || freq_hz < 100000.0 || freq_hz > 3000000000.0) return;

    char call[SPOT_CALL_MAX + 1], grid[SPOT_GRID_MAX + 1];
    if (!parse_msg(text, mode, call, sizeof call, grid, sizeof grid)) return;
    if (grid[0] == '\0' && mode[0] == 'W') return; /* WSPR always carries a grid */

    uint32_t now = (uint32_t)time(NULL);
    if (xSemaphoreTake(s_mu, pdMS_TO_TICKS(100)) != pdTRUE) return;
    bool fresh = dedup_check_and_add(mode, call, (uint32_t)(freq_hz + 0.5), now);
    if (fresh && s_qn < QUEUE_N) {
        spot_t *s = &s_q[s_qh];
        snprintf(s->call, sizeof s->call, "%s", call);
        snprintf(s->grid, sizeof s->grid, "%s", grid);
        snprintf(s->mode, sizeof s->mode, "%s", mode);
        s->freq = (uint32_t)(freq_hz + 0.5);
        s->snr = (int8_t)(snr_db > 127 ? 127 : snr_db < -128 ? -128 : snr_db);
        s->t = now;
        s_qh = (s_qh + 1) % QUEUE_N;
        s_qn++;
    }
    xSemaphoreGive(s_mu);
}

/* ---- sender task ------------------------------------------------------------------ */

/* Build one datagram from up to `max_spots` queued spots (caller holds s_mu).
 * Returns the datagram length, 0 if nothing to send. */
static size_t build_dgram(uint8_t *buf, int max_spots)
{
    int tail = (s_qh - s_qn + QUEUE_N) % QUEUE_N;
    int n = s_qn < max_spots ? s_qn : max_spots;

    uint8_t *p = buf;
    *p++ = 0x00; *p++ = 0x0A;             /* version */
    uint8_t *len_f = p; p += 2;           /* total length, patched below */
    p = put32(p, (uint32_t)time(NULL));   /* export time */
    p = put32(p, s_seq);                  /* sequence = count of REPORTS */
    p = put32(p, s_rid);                  /* observation domain id */

    memcpy(p, TPL_RX, sizeof TPL_RX); p += sizeof TPL_RX;
    memcpy(p, TPL_TX, sizeof TPL_TX); p += sizeof TPL_TX;

    /* receiver record — per spec, the length code includes header AND padding */
    {
        uint8_t *rec = p;
        p += 4; /* set id + length */
        station_settings_t st;
        station_settings_get(&st);
        p = put_str(p, st.callsign);
        p = put_str(p, st.grid);
        p = put_str(p, PSK_SW_NAME);
        p = put_str(p, PSK_RIG);
        size_t plen = (size_t)(p - rec);
        uint8_t pad = (uint8_t)((4 - (plen & 3)) & 3);
        while (pad--) *p++ = 0;
        rec[0] = 0x99; rec[1] = 0x92;
        put16(rec + 2, (uint16_t)(p - rec));
    }

    /* sender records (same length convention; iMD byte = 0, not measured) */
    {
        uint8_t *rec = p;
        p += 4;
        for (int i = 0; i < n; i++) {
            spot_t *s = &s_q[(tail + i) % QUEUE_N];
            p = put_str(p, s->call);
            p = put32(p, s->freq);
            *p++ = (uint8_t)s->snr;
            *p++ = 0; /* iMD (intermodulation distortion): not measured */
            p = put_str(p, s->mode);
            *p++ = 1; /* informationSource: automated software decoding */
            p = put_str(p, s->grid);
            p = put32(p, s->t);
        }
        size_t plen = (size_t)(p - rec);
        uint8_t pad = (uint8_t)((4 - (plen & 3)) & 3);
        while (pad--) *p++ = 0;
        rec[0] = 0x99; rec[1] = 0x93;
        put16(rec + 2, (uint16_t)(p - rec));
        tail = (tail + n) % QUEUE_N;
        s_qn -= n;
        s_seq += n; /* sequence counts reports, not packets */
    }

    put16(len_f, (uint16_t)(p - buf));
    return (size_t)(p - buf);
}

static bool send_dgram(const uint8_t *buf, size_t len)
{
    struct addrinfo hints = {0}, *res = NULL;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(PSK_HOST, STR(PSK_PORT), &hints, &res) != 0 || !res) return false;

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    bool ok = false;
    if (fd >= 0) {
        ok = sendto(fd, buf, len, 0, res->ai_addr, res->ai_addrlen) == (ssize_t)len;
        close(fd);
    }
    freeaddrinfo(res);
    return ok;
}

static void sender_task(void *arg)
{
    (void)arg;
    static uint8_t buf[MAX_DGRAM + 64];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (!s_enabled || !wifi_net_connected() || !wifi_net_time_ok()) continue;

        uint32_t now = (uint32_t)time(NULL);
        xSemaphoreTake(s_mu, portMAX_DELAY);
        /* flush when the batch is big enough or the oldest spot has aged */
        uint32_t oldest = s_qn ? s_q[(s_qh - s_qn + QUEUE_N) % QUEUE_N].t : now;
        if (s_qn == 0 || (s_qn < PSK_FLUSH_MIN_SPOTS && now - oldest < PSK_FLUSH_MAX_AGE_S)) {
            xSemaphoreGive(s_mu);
            continue;
        }
        /* worst-case spot: 1+14 call, 4 freq, 1 snr, 1+6 mode, 1 src, 1+6 grid,
         * 4 time ≈ 39 B; budget = MAX_DGRAM − 112 B header/templates − rx record */
        int max_spots = (MAX_DGRAM - 160) / 40;
        if (max_spots < 1) max_spots = 1;
        size_t len = build_dgram(buf, max_spots);
        xSemaphoreGive(s_mu);

        if (len && send_dgram(buf, len)) {
            ESP_LOGI(TAG, "datagram sent (%u bytes):", (unsigned)len);
            ESP_LOG_BUFFER_HEX(TAG, buf, len < 160 ? len : 160); /* for format checks */
            s_sent++;
        } else if (len) {
            ESP_LOGW(TAG, "send failed");
        }
    }
}

/* ---- public API --------------------------------------------------------------------- */

static void nvs_save_enabled(void)
{
    nvs_handle_t h;
    if (nvs_open("psk", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "en", s_enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool psk_enabled(void) { return s_enabled; }

void psk_set_enabled(bool on)
{
    s_enabled = on;
    nvs_save_enabled();
    ESP_LOGI(TAG, "PSK Reporter upload %s", on ? "ON" : "OFF");
}

int psk_status(void)
{
    if (!s_enabled) return 0;
    return (wifi_net_connected() && wifi_net_time_ok()) ? 2 : 1;
}

uint32_t psk_spots_sent(void) { return s_sent; }

void psk_init(void)
{
    s_mu = xSemaphoreCreateMutex();
    s_rid = esp_random();

    uint8_t en = PSK_ENABLED_DEFAULT;
    nvs_handle_t h;
    if (nvs_open("psk", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "en", &en);
        nvs_close(h);
    }
    s_enabled = en != 0;

    xTaskCreatePinnedToCore(sender_task, "psk_tx", 6144, NULL, 2, NULL, 0);
    ESP_LOGI(TAG, "uploader started (%s), upload %s", PSK_CALLSIGN, s_enabled ? "ON" : "OFF");
}
