/* Horus Binary v1 receive path. LGPL-2.1 modem code is in horuslib/. */
#include "horus_decoder.h"
#include "horus_v3.h"
#include "horuslib/horus_api.h"
#include "horuslib/fsk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHUNK 128
#define FS 8000
#define IQ_SCALE 180.0f
typedef struct { int16_t iq[CHUNK*2]; } iq_chunk_t;
static QueueHandle_t q;
static volatile bool active;
static uint32_t generation;
static iq_chunk_t pending;
static unsigned fill, decim;
/* 48 kHz -> 8 kHz: anti-alias FIR, cutoff 1.8 kHz. */
#define HORUS_FIR_TAPS 97
static float fir_h[HORUS_FIR_TAPS];
static float fir_i[HORUS_FIR_TAPS], fir_q[HORUS_FIR_TAPS];
static unsigned fir_pos;
static bool fir_ready;

static void fir_init(void)
{
    float sum = 0.0f;
    const float fc = 1800.0f / 48000.0f;
    for (int k = 0; k < HORUS_FIR_TAPS; ++k) {
        int n = k - (HORUS_FIR_TAPS - 1) / 2;
        float sinc = (n == 0) ? 2.0f * fc :
            sinf(2.0f * (float)M_PI * fc * n) / ((float)M_PI * n);
        float window = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * k / (HORUS_FIR_TAPS - 1));
        fir_h[k] = sinc * window;
        sum += fir_h[k];
    }
    for (int k = 0; k < HORUS_FIR_TAPS; ++k) fir_h[k] /= sum;
    fir_ready = true;
}
static portMUX_TYPE guard = portMUX_INITIALIZER_UNLOCKED;
static uint32_t frames, drops;
static volatile uint32_t iq_samples, modem_calls;
static volatile int tone_hz[4];
static volatile unsigned int iq_level;
static horus_v1_telemetry_t telemetry;
static char raw_hex[260];
static unsigned raw_len, raw_version, parse_ok;
static char last[80] = "Waiting for CRC-valid Horus v1 packet";

/* The modem returns 22 payload bytes encoded as 44 ASCII hex characters.
 * Layout from upstream horusdemodlib decoder.py: <BH3sffHBBbBH. */
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static bool parse_v1(const char *hex, horus_v1_telemetry_t *out)
{
    if (strlen(hex) != 44) return false;
    uint8_t p[22];
    for (int j = 0; j < 22; ++j) {
        int hi = hexval(hex[j*2]), lo = hexval(hex[j*2+1]);
        if (hi < 0 || lo < 0) return false;
        p[j] = (uint8_t)((hi << 4) | lo);
    }
    uint32_t lat_bits = (uint32_t)p[6] | ((uint32_t)p[7]<<8) | ((uint32_t)p[8]<<16) | ((uint32_t)p[9]<<24);
    uint32_t lon_bits = (uint32_t)p[10] | ((uint32_t)p[11]<<8) | ((uint32_t)p[12]<<16) | ((uint32_t)p[13]<<24);
    float lat, lon;
    memcpy(&lat, &lat_bits, sizeof(lat));
    memcpy(&lon, &lon_bits, sizeof(lon));
    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->version = 1;
    out->payload_id = p[0];
    out->sequence = (unsigned)p[1] | ((unsigned)p[2] << 8);
    out->hour = p[3]; out->minute = p[4]; out->second = p[5];
    out->latitude = lat; out->longitude = lon;
    out->altitude = (unsigned)p[14] | ((unsigned)p[15] << 8);
    out->speed = p[16]; out->satellites = p[17];
    out->temperature = (int8_t)p[18];
    out->battery_raw = p[19];
    return true;
}


/* Horus Binary v2, 32-byte packet, upstream decoder.py:
 * <HH3sffHBBbB9sH (little-endian). CRC is checked by horusdemodlib.
 * Unlike v1, payload ID is 16-bit and the GPS floats start at byte 7. */
static bool parse_v2_32(const char *hex, horus_v1_telemetry_t *out)
{
    if (strlen(hex) != 64) return false;
    uint8_t p[32];
    for (int j = 0; j < 32; ++j) {
        int hi = hexval(hex[j * 2]), lo = hexval(hex[j * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        p[j] = (uint8_t)((hi << 4) | lo);
    }
    uint32_t lat_bits = (uint32_t)p[7] | ((uint32_t)p[8] << 8) |
        ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);
    uint32_t lon_bits = (uint32_t)p[11] | ((uint32_t)p[12] << 8) |
        ((uint32_t)p[13] << 16) | ((uint32_t)p[14] << 24);
    float lat, lon;
    memcpy(&lat, &lat_bits, sizeof(lat));
    memcpy(&lon, &lon_bits, sizeof(lon));
    if (!isfinite(lat) || !isfinite(lon) || fabsf(lat) > 90 || fabsf(lon) > 180)
        return false;
    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->version = 2;
    out->payload_id = (unsigned)p[0] | ((unsigned)p[1] << 8);
    out->sequence = (unsigned)p[2] | ((unsigned)p[3] << 8);
    out->hour = p[4]; out->minute = p[5]; out->second = p[6];
    out->latitude = lat; out->longitude = lon;
    out->altitude = (unsigned)p[15] | ((unsigned)p[16] << 8);
    out->speed = p[17]; out->satellites = p[18];
    out->temperature = (int8_t)p[19];
    out->battery_raw = p[20];
    return true;
}

static void worker(void *unused)
{
    (void)unused;
    struct horus *h = horus_open_advanced_sample_rate(HORUS_MODE_BINARY_V1,100,270,FS,8);
    if (!h) { ESP_LOGE("HORUS", "Cannot create modem"); vTaskDelete(NULL); return; }
    /* Complex zero-IF IQ contains negative and positive FSK tones. */
    h->fsk->est_min = -2500;
    h->fsk->est_max = 2500;
    int cap = horus_get_max_demod_in(h);
    short *samples = calloc((size_t)cap * 2 + 64, sizeof(short));
    char *packet = calloc((size_t)horus_get_max_ascii_out_len(h) + 2,1);
    if (!samples || !packet) { ESP_LOGE("HORUS", "Out of memory"); free(samples); free(packet); horus_close(h); vTaskDelete(NULL); return; }
    uint32_t seen = generation;
    int have = 0;
    iq_chunk_t chunk;
    for (;;) {
        if (xQueueReceive(q,&chunk,pdMS_TO_TICKS(100)) != pdTRUE) continue;
        if (!active) { have = 0; continue; }
        if (seen != generation) { /* reset modem on mode entry */
            horus_close(h);
            h = horus_open_advanced_sample_rate(HORUS_MODE_BINARY_V1,100,270,FS,8);
            if (!h) { ESP_LOGE("HORUS", "Cannot reset modem"); break; }
            h->fsk->est_min = -2500;
            h->fsk->est_max = 2500;
            have = 0; seen = generation;
        }
        for (int j=0;j<CHUNK;j++) {
            samples[2*have] = chunk.iq[2*j];
            samples[2*have+1] = chunk.iq[2*j+1];
            have++;
            int nin = (int)horus_nin(h);
            if (have >= nin) {
                packet[0] = 0;
                int got_packet = horus_rx(h,packet,samples,1);
                modem_calls++;
                /* f_est is updated by the 4FSK demodulator. Display the
                 * detected baseband tones to distinguish signal from noise. */
                for (int k = 0; k < 4; ++k)
                    tone_hz[k] = (int)h->fsk->f2_est[k];
                if (got_packet && horus_crc_ok(h)) {
                    horus_v1_telemetry_t decoded;
                    bool parsed = (h->version == 1) ? parse_v1(packet, &decoded) :
                                  (h->version == 2) ? parse_v2_32(packet, &decoded) :
                                  (h->version == 3) ? horus_v3_parse(packet, &decoded) : false;
                    portENTER_CRITICAL(&guard);
                    if (parsed) telemetry = decoded;
                    raw_len = (unsigned)strlen(packet);
                    raw_version = (unsigned)h->version;
                    parse_ok = parsed;
                    snprintf(raw_hex, sizeof(raw_hex), "%s", packet);
                    frames++;
                    snprintf(last,sizeof(last),"CRC OK #%lu %.56s",(unsigned long)frames,packet);
                    portEXIT_CRITICAL(&guard);
                    ESP_LOGI("HORUS", "v%u CRC OK: %s", (unsigned)h->version, packet);
                }
                have = 0;
            }
        }
    }
    free(samples); free(packet); vTaskDelete(NULL);
}

void horus_decoder_start(void)
{
    if (q) return;
    q = xQueueCreate(24,sizeof(iq_chunk_t));
    if (!q) return;
    if (xTaskCreate(worker,"horus_v1",20480,NULL,4,NULL) != pdPASS) {
        vQueueDelete(q); q=NULL;
    }
}

void horus_decoder_reset(void)
{
    if (!q) horus_decoder_start();
    active = false;
    if (q) xQueueReset(q);
    fill = decim = fir_pos = 0;
    memset(fir_i, 0, sizeof(fir_i));
    memset(fir_q, 0, sizeof(fir_q));
    if (!fir_ready) fir_init();
    iq_samples = modem_calls = iq_level = 0;
    portENTER_CRITICAL(&guard);
    memset(&telemetry, 0, sizeof(telemetry));
    raw_hex[0] = 0; raw_len = raw_version = parse_ok = 0;
    portEXIT_CRITICAL(&guard);
    for (int k=0;k<4;k++) tone_hz[k]=0;
    generation++;
    active = true;
}

void horus_decoder_feed_iq(float i,float qv)
{
    if (!active || !q) return;
    /* Proper anti-alias filtering before decimation: old single-pole filter
     * passed too much 4-24 kHz noise into the 8 kHz modem. */
    fir_i[fir_pos] = i;
    fir_q[fir_pos] = qv;
    if (++fir_pos == HORUS_FIR_TAPS) fir_pos = 0;
    if (++decim < 6) return;
    decim = 0;
    float fi = 0.0f, fq = 0.0f;
    unsigned idx = fir_pos;
    for (int k = 0; k < HORUS_FIR_TAPS; ++k) {
        if (idx == 0) idx = HORUS_FIR_TAPS;
        --idx;
        fi += fir_h[k] * fir_i[idx];
        fq += fir_h[k] * fir_q[idx];
    }
    float a = fi * IQ_SCALE, b = fq * IQ_SCALE;
    if (a > 32767) a = 32767;
    if (a < -32768) a = -32768;
    if (b > 32767) b = 32767;
    if (b < -32768) b = -32768;
    iq_samples++;
    /* Smoothed absolute amplitude; updated at 8 ksample/s. */
    unsigned int amp = (unsigned int)(fabsf(a) + fabsf(b));
    iq_level = (iq_level * 31u + amp) / 32u;
    pending.iq[2*fill]=(int16_t)a;
    pending.iq[2*fill+1]=(int16_t)b;
    if (++fill == CHUNK) {
        if (xQueueSend(q,&pending,0) != pdTRUE) { portENTER_CRITICAL(&guard); drops++; portEXIT_CRITICAL(&guard); }
        fill=0;
    }
}

void horus_decoder_status(char *dst,unsigned size)
{
    if (!dst || !size) return;
    portENTER_CRITICAL(&guard);
    snprintf(dst,size,"V1:%lu IQ:%lu DEM:%lu L:%u F:%d/%d/%d/%d D:%lu",
             (unsigned long)frames, (unsigned long)iq_samples,
             (unsigned long)modem_calls, iq_level,
             tone_hz[0],tone_hz[1],tone_hz[2],tone_hz[3],
             (unsigned long)drops);
    portEXIT_CRITICAL(&guard);
}

bool horus_decoder_get_telemetry(horus_v1_telemetry_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&guard);
    *out = telemetry;
    portEXIT_CRITICAL(&guard);
    return out->valid;
}

void horus_decoder_raw(char *dst, unsigned size, unsigned *version, unsigned *length, bool *parsed)
{
    portENTER_CRITICAL(&guard);
    if (dst && size) snprintf(dst, size, "%s", raw_hex);
    if (version) *version = raw_version;
    if (length) *length = raw_len;
    if (parsed) *parsed = parse_ok != 0;
    portEXIT_CRITICAL(&guard);
}
