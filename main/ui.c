/*
 * ui.c — landscape touch UI for Tab5-SDR.
 *
 * Rendered into a 640x360 landscape RGB565 buffer (LBUF); the P4 PPA scales x2
 * and rotates 90° onto the 720x1280 panel (half-res keeps the PPA cheap).
 * Touch maps back: lx = (1280 - panel_y)/2,  ly = panel_x/2.
 *
 * Controls: drag spectrum/waterfall to scan fast; [-1][-.1][+.1][+1] tune;
 * mode row WFM/NFM/AM/USB/LSB/CW/FT8/WSPR, RS41 button (press again = centre on peak);
 * HF/VHF/UHF band buttons; MENU button: time-scheduled band hopping for
 * FT8/WSPR (menu: OFF / FT8 PLAN / WSPR PLAN / USER PLAN / editor / SCREEN OFF);
 * volume +/- ; squelch +/-. The SCREEN OFF row sets the display-off (screensaver)
 * delay; after the idle timeout the backlight turns off and any touch wakes it.
 */
#include "ui.h"
#include "web_mirror.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include "rs41_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "station_settings.h"
#include "wifi_net.h"
#include "driver/ppa.h"
#include "driver/temperature_sensor.h"

#include "bsp_tab5.h"
#include "sdr.h"
#include "dsp.h"
#include "rs41.h"
#include "rs41_font.h"
#include "ft8.h"
#include "ft4.h"
#include "wspr.h"
#include "audio.h"
#include "bookmarks.h"
#include "psk.h"
#include "bandplan.h"
#include "si5351.h"
#include "lora_aprs.h"
#include "aprs.h"
/* Stream pause/resume API implemented in sdr.c. */
extern bool sdr_stream_pause(void);
extern bool sdr_stream_resume(void);
#include "horus.h"
#include "horus_decoder.h"
#include "horus_payload_ids.h"

static const char *TAG = "ui";

/* Internal chip temperature sensor (not an individual CPU core). */
static temperature_sensor_handle_t s_temp_sensor = NULL;
static float s_cpu_temp_c = 0.0f;
static bool s_cpu_temp_valid = false;

static void cpu_temp_init(void)
{
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&cfg, &s_temp_sensor);
    if (err == ESP_OK) err = temperature_sensor_enable(s_temp_sensor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CPU temperature sensor unavailable: %s", esp_err_to_name(err));
        if (s_temp_sensor) {
            temperature_sensor_uninstall(s_temp_sensor);
            s_temp_sensor = NULL;
        }
    }
}

static void cpu_temp_sample(void)
{
    if (!s_temp_sensor) return;
    float c;
    if (temperature_sensor_get_celsius(s_temp_sensor, &c) == ESP_OK) {
        s_cpu_temp_c = c;
        s_cpu_temp_valid = true;
    }
}


#define LW 640
#define LH 360
#define PANEL_W 720
#define PANEL_H 1280
#define SCALE 2

#define SPEC_Y0 42
#define SPEC_H  66
#define WF_Y0   (SPEC_Y0 + SPEC_H + 4) /* 112 */
#define WF_H    98                     /* 112..210 */
#define CTRL_Y0 214

#define DB_MIN 44.0f
#define DB_MAX 88.0f

#define FREQ_MIN 500000u
#define FREQ_MAX 1766000000u

/* Drag tuning: full canvas width ≈ 100 MHz (fast band scanning). */
#define DRAG_HZ_PER_PX (10000 / LW) // 7 nula za 10Mhz
#define ZOOM_MIN_HZ 10000u /* visual zoom only; DSP sample rate is unchanged */

#define SQ_OFFSET 64.0f
#define SQ_STEP   5
#define VOL_STEP  5

#define RGB565(r, g, b) \
    ((uint16_t)(((((r) >> 3) & 0x1F) << 11) | ((((g) >> 2) & 0x3F) << 5) | (((b) >> 3) & 0x1F)))
#define COL_BG     RGB565(8, 12, 28)
#define COL_PANEL  RGB565(18, 22, 40)
#define COL_TEXT   RGB565(210, 220, 240)
#define COL_ACCENT RGB565(0, 220, 120)
#define COL_LINE   RGB565(255, 210, 0)
#define COL_BTN    RGB565(40, 60, 110)
#define COL_BTN_HI RGB565(80, 110, 180)
#define COL_TRACK  RGB565(50, 56, 78)

/* Startup splash: full-screen welcome banner shown for a few seconds after
 * boot. Edit the callsign/text to your own. */
#define SPLASH_TITLE "M5-SDR"
/* Shared station identity: SETTINGS / NVS is the single source of truth. */
static station_settings_t s_station;
static bool s_station_loaded;
static void station_identity_reload(void)
{
    station_settings_get(&s_station);
    s_station_loaded = true;
}
static const char *station_call(void)
{
    if (!s_station_loaded) station_identity_reload();
    return s_station.callsign;
}
static const char *station_grid(void)
{
    if (!s_station_loaded) station_identity_reload();
    return s_station.grid;
}
#define SPLASH_MS    2500

static uint16_t *LBUF;
static ppa_client_handle_t s_ppa;
static int s_fb_index;

static uint32_t s_center;
static uint32_t s_view_span; /* 0 means full DSP spectrum width */
static int s_volume = AUDIO_VOL_DEFAULT;
static int s_squelch = -60;
static int s_mode = DSP_MODE_WFM;
static const char *s_label = "WFM broadcast";
static int s_bm_hf = -1, s_bm_vhf = 0, s_bm_uhf = 0; /* per-band preset cursor */
static uint32_t s_rate_cur = 1152000; /* RTL2832 sample rate currently selected */

/* Tuning step (Hz) applied by the [-]/[+] buttons; STEP button cycles it. */
static const uint32_t STEPS[] = {100, 500, 1000, 5000, 25000, 100000, 1000000};
#define N_STEPS ((int)(sizeof(STEPS) / sizeof(STEPS[0])))
static int s_step_i = 6; /* default 25 kHz */

/* Tuner gain steps in 0.1 dB; -1 = automatic. Manual gain avoids the tuner AGC
 * turning the gain down because of strong signals elsewhere on the band, which
 * would leave a weak signal (e.g. a sonde) only a few ADC counts high. */
static const int GAINS[] = {-1, 200, 280, 364, 402, 434, 480, 496};
#define N_GAINS (int)(sizeof GAINS / sizeof GAINS[0])
static int s_gain_i = 0;

static void step_label(char *out, size_t n)
{
    uint32_t s = STEPS[s_step_i];
    if (s >= 1000000) snprintf(out, n, "%uM", (unsigned)(s / 1000000));
    else if (s % 1000 == 0) snprintf(out, n, "%uk", (unsigned)(s / 1000));
    else if (s < 1000) snprintf(out, n, "%u", (unsigned)s);
    else snprintf(out, n, "%u.%02uk", (unsigned)(s / 1000), (unsigned)((s % 1000) / 10));
}

/* ---- primitives ---------------------------------------------------------- */

static void rect(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LW) w = LW - x;
    if (y + h > LH) h = LH - y;
    for (int j = 0; j < h; j++) {
        uint16_t *row = LBUF + (size_t)(y + j) * LW + x;
        for (int i = 0; i < w; i++) row[i] = c;
    }
}

static const uint8_t *glyph(char c)
{
    static const uint8_t G[][7] = {
        {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
        {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
        {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
        {0x0E,0x11,0x10,0x1E,0x11,0x11,0x0E}, {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
        {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, {0x0E,0x11,0x11,0x0F,0x01,0x11,0x0E},
        {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
        {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
        {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
        {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
        {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
        {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}, /* + */
        {0x1F,0x11,0x11,0x11,0x11,0x11,0x1F}, /* D (also generic box) */
        {0x1F,0x10,0x1C,0x10,0x10,0x10,0x10}, /* F */
        {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}, /* W */
        {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, /* I */
        {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, /* G (NOAA/misc) */
        {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, /* N */
        {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, /* T */
        {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, /* E */
        {0x10,0x10,0x12,0x14,0x18,0x14,0x12}, /* k */
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* space */
        {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, /* U  (31) */
        {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, /* B  (32) */
        {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, /* A  (33) */
        {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, /* C  (34) */
        {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, /* X  (35) */
        {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, /* Y  (36) */
        {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}, /* :  (37) */
        {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, /* R  (38) */
        {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, /* P  (39) */
        {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, /* J  (40) */
        {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, /* K  (41) */
        {0x01,0x01,0x02,0x04,0x08,0x10,0x10}, /* /  (42) */
        {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, /* <  (43) */
        {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, /* >  (44) */
    };
    static const uint8_t lowercase[26][7] = {
        {0x00,0x0E,0x01,0x0F,0x11,0x0F,0x00},
        {0x00,0x10,0x10,0x1E,0x11,0x1E,0x00},
        {0x00,0x00,0x0E,0x10,0x10,0x0E,0x00},
        {0x00,0x01,0x01,0x0F,0x11,0x0F,0x00},
        {0x00,0x00,0x0E,0x1F,0x10,0x0E,0x00},
        {0x00,0x06,0x08,0x1C,0x08,0x08,0x00},
        {0x00,0x00,0x0F,0x11,0x0F,0x01,0x00},
        {0x00,0x10,0x10,0x1E,0x11,0x11,0x00},
        {0x00,0x04,0x00,0x0C,0x04,0x0E,0x00},
        {0x00,0x02,0x00,0x02,0x12,0x0C,0x00},
        {0x00,0x10,0x12,0x14,0x18,0x16,0x00},
        {0x00,0x0C,0x04,0x04,0x04,0x0E,0x00},
        {0x00,0x00,0x1A,0x15,0x15,0x15,0x00},
        {0x00,0x00,0x1E,0x11,0x11,0x11,0x00},
        {0x00,0x00,0x0E,0x11,0x11,0x0E,0x00},
        {0x00,0x00,0x1E,0x11,0x1E,0x10,0x00},
        {0x00,0x00,0x0F,0x11,0x0F,0x01,0x00},
        {0x00,0x00,0x16,0x18,0x10,0x10,0x00},
        {0x00,0x00,0x0F,0x18,0x03,0x1E,0x00},
        {0x00,0x08,0x1C,0x08,0x08,0x06,0x00},
        {0x00,0x00,0x11,0x11,0x13,0x0D,0x00},
        {0x00,0x00,0x11,0x11,0x0A,0x04,0x00},
        {0x00,0x00,0x11,0x15,0x15,0x0A,0x00},
        {0x00,0x00,0x11,0x0A,0x04,0x1B,0x00},
        {0x00,0x00,0x11,0x11,0x0F,0x01,0x00},
        {0x00,0x00,0x1F,0x02,0x04,0x1F,0x00}
    };
    static const uint8_t underscore[7] = {0,0,0,0,0,0,0x1F};
    static const uint8_t at_sign[7] = {0x0E,0x11,0x17,0x15,0x17,0x10,0x0E};
    if (c >= 'a' && c <= 'z') return lowercase[c - 'a'];
    if (c == '_') return underscore;
    if (c == '@') return at_sign;
    switch (c) {
    case '0' ... '9': return G[c - '0'];
    case 'M': return G[10];  case 'H': return G[11];  case 'z': return G[12];
    case 'V': return G[13];  case 'O': return G[14];  case 'L': return G[15];
    case 'S': return G[16];  case 'Q': return G[17];  case '-': return G[18];
    case '.': return G[19];  case '+': return G[20];  case 'D': return G[21];
    case 'F': return G[22];  case 'W': return G[23];  case 'I': return G[24];
    case 'G': return G[25];  case 'N': return G[26];  case 'T': return G[27];
    case 'E': return G[28];  case 'k': return G[29];
    case 'U': return G[31];  case 'B': return G[32];  case 'A': return G[33];
    case 'C': return G[34];  case 'X': return G[35];  case 'Y': return G[36];
    case ':': return G[37];  case 'R': return G[38];  case 'P': return G[39];
    case 'J': return G[40];  case 'K': return G[41];  case 'Z': return G[12];
    case '/': return G[42];  case '<': return G[43];  case '>': return G[44];
    default:  return G[30];
    }
}

static void text(int x, int y, int sc, uint16_t c, const char *s)
{
    for (; *s; s++) {
        const uint8_t *g = glyph(*s);
        for (int row = 0; row < 7; row++)
            for (int col = 0; col < 5; col++)
                if (g[row] & (1 << (4 - col))) rect(x + col * sc, y + row * sc, sc, sc, c);
        x += 6 * sc;
    }
}
/* HORUS-only font: 1.5x the original bitmap, with comfortable line spacing. */
static void horus_text(int x, int y, uint16_t c, const char *s)
{
    for (; *s; ++s) {
        const uint8_t *g = glyph(*s);
        for (int row = 0; row < 7; ++row) {
            int y0 = y + (row * 3) / 2;
            int y1 = y + ((row + 1) * 3 + 1) / 2;
            for (int col = 0; col < 5; ++col) {
                if (g[row] & (1 << (4 - col))) {
                    int x0 = x + (col * 3) / 2;
                    int x1 = x + ((col + 1) * 3 + 1) / 2;
                    rect(x0, y0, x1 - x0, y1 - y0, c);
                }
            }
        }
        x += 9;
    }
}

static int text_w(const char *s, int sc) { return (int)strlen(s) * 6 * sc; }

static void seg7_digit(int x, int y, int dw, int dh, int t, int d, uint16_t c)
{
    static const uint8_t M[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
    int hh = (dh - 3 * t) / 2, m = M[d];
    if (m & 1)  rect(x + t, y, dw - 2 * t, t, c);
    if (m & 2)  rect(x + dw - t, y + t, t, hh, c);
    if (m & 4)  rect(x + dw - t, y + 2 * t + hh, t, hh, c);
    if (m & 8)  rect(x + t, y + 2 * t + 2 * hh, dw - 2 * t, t, c);
    if (m & 16) rect(x, y + 2 * t + hh, t, hh, c);
    if (m & 32) rect(x, y + t, t, hh, c);
    if (m & 64) rect(x + t, y + t + hh, dw - 2 * t, t, c);
}

/* Anti-aliased RS41-only text; blends with the RGB565 framebuffer. */
static void rs41_smooth_text(int x, int y, uint16_t fg, const char *str)
{
    int fr = ((fg >> 11) & 31) * 255 / 31;
    int fgc = ((fg >> 5) & 63) * 255 / 63;
    int fb = (fg & 31) * 255 / 31;
    for (; *str && x < LW; ++str, x += RSFONT_W) {
        unsigned char ch = (unsigned char)*str;
        if (ch < 32 || ch > 126) ch = '?';
        const uint8_t *glyph_a = rsfont_alpha + (ch - 32) * RSFONT_W * RSFONT_H;
        for (int yy = 0; yy < RSFONT_H; ++yy) {
            int py = y + yy;
            if (py < 0 || py >= LH) continue;
            for (int xx = 0; xx < RSFONT_W; ++xx) {
                int px = x + xx;
                if (px < 0 || px >= LW) continue;
                unsigned a = glyph_a[yy * RSFONT_W + xx];
                if (!a) continue;
                uint16_t *dst = &LBUF[(size_t)py * LW + px];
                if (a == 255) { *dst = fg; continue; }
                uint16_t bg = *dst;
                int br = ((bg >> 11) & 31) * 255 / 31;
                int bgc = ((bg >> 5) & 63) * 255 / 63;
                int bb = (bg & 31) * 255 / 31;
                int r = (fr * (int)a + br * (255 - (int)a) + 127) / 255;
                int g = (fgc * (int)a + bgc * (255 - (int)a) + 127) / 255;
                int b = (fb * (int)a + bb * (255 - (int)a) + 127) / 255;
                *dst = RGB565(r, g, b);
            }
        }
    }
}

/* ---- mode helpers -------------------------------------------------------- */

static uint16_t mode_color(int m)
{
    switch (m) {
    case DSP_MODE_NFM: return RGB565(255, 160, 0);
    case DSP_MODE_AM:  return RGB565(90, 190, 255);
    case DSP_MODE_USB:
    case DSP_MODE_LSB: return RGB565(255, 120, 200);
    case DSP_MODE_CW:  return RGB565(200, 160, 255);
    case DSP_MODE_RS41: return RGB565(255, 90, 90);
    case DSP_MODE_FT8:  return RGB565(60, 220, 200);
    case DSP_MODE_FT4:  return RGB565(80, 210, 170);
    case DSP_MODE_WSPR: return RGB565(240, 170, 80);
    case DSP_MODE_HORUS: return RGB565(140, 220, 120);
    case DSP_MODE_APRS: return RGB565(90, 220, 245);
    default:           return COL_ACCENT; /* WFM */
    }
}

/* ---- header: freq + mode/label + S-meter --------------------------------- */

static void draw_cpu_temp(void)
{
    char b[24];
    rect(535, 27, 102, 13, COL_BG);
    if (s_cpu_temp_valid)
        snprintf(b, sizeof b, "CPU %.1f C", (double)s_cpu_temp_c);
    else
        snprintf(b, sizeof b, "CPU --.- C");
    uint16_t color = s_cpu_temp_valid && s_cpu_temp_c >= 70.0f
        ? RGB565(255, 100, 80) : COL_TEXT;
    text(541, 29, 1, color, b);
}

static uint32_t view_span_hz(void);

/* RS41 auto scanner. Run entirely in the UI task; the RS41 decoder and
 * SD logger keep running in their existing tasks. */
typedef enum { RS_SCAN_OFF, RS_SCAN_SWEEP, RS_SCAN_VERIFY, RS_SCAN_LOCKED } rs_scan_state_t;
static rs_scan_state_t s_rs_scan = RS_SCAN_OFF;
static uint32_t s_rs_scan_since;
static uint32_t s_rs_scan_last_frame;
static uint32_t s_rs_scan_seq;
static int s_rs_scan_step;
static const char *rs_scan_status(void)
{
    switch (s_rs_scan) {
    case RS_SCAN_SWEEP: return "RS41 SCANNING";
    case RS_SCAN_VERIFY: return "RS41 VERIFY";
    case RS_SCAN_LOCKED: return "RS41 LOCKED";
    default: return "MANUAL";
    }
}
static void rs_scan_stop(void)
{
    if (s_rs_scan != RS_SCAN_OFF) ESP_LOGI("rs41_scan", "Stopped");
    s_rs_scan = RS_SCAN_OFF;
    if (s_mode == DSP_MODE_RS41) s_label = "RS41 MANUAL";
}
static void draw_header(void)
{
    rect(0, 0, LW, SPEC_Y0, COL_BG);

    /* frequency, centred, 3-decimal MHz */
    char buf[16];
    snprintf(buf, sizeof buf, "%.4f", s_center / 1e6); /* 100 Hz resolution (SSB/CW) */
    const int dw = 17, dh = 28, t = 3, gap = 4, dotw = 8;
    int width = 0;
    for (const char *p = buf; *p; p++) width += (*p == '.') ? dotw : (dw + gap);
    int x = (LW - width) / 2, y = 8;
    for (const char *p = buf; *p; p++) {
        if (*p == '.') { rect(x + 2, y + dh - 5, 5, 5, COL_LINE); x += dotw; }
        else { seg7_digit(x, y, dw, dh, t, *p - '0', COL_LINE); x += dw + gap; }
    }

    /* mode + bookmark label, top-left */
    text(6, 4, 2, mode_color(s_mode), dsp_mode_name(s_mode));
    /* Zoom span beside the active mode, outside the waterfall. */
    uint32_t visible = view_span_hz();
    char span_label[32];
    if (visible >= 1000000)
        snprintf(span_label, sizeof span_label, "SPAN %.2fM", visible / 1e6);
    else
        snprintf(span_label, sizeof span_label, "SPAN %luK", (unsigned long)(visible / 1000));
    text(100, 7, 1, COL_TEXT, span_label);
    text(6, 24, 1, s_rs_scan == RS_SCAN_LOCKED ? COL_LINE : COL_TEXT,
         s_rs_scan == RS_SCAN_OFF ? s_label : rs_scan_status());

    /* S-meter top-right, -60..+60 relative scale */
    int mx = 470, my = 6, mw = 160, mh = 11;
    rect(mx, my, mw, mh, COL_TRACK);
    float tt = (dsp_signal_db() - SQ_OFFSET + 60.0f) / 120.0f;
    if (tt < 0) tt = 0;
    if (tt > 1) tt = 1;
    rect(mx, my, (int)(mw * tt), mh, COL_ACCENT);
    if (s_squelch > -60) {
        float ts = (s_squelch + 60.0f) / 120.0f;
        rect(mx + (int)(mw * ts), my - 3, 2, mh + 6, RGB565(240, 80, 80));
    }
    text(mx, my + mh + 2, 1, COL_TEXT, "SIG");
    draw_cpu_temp();
    /* IP in the upper-left corner of the main screen (small text). */
    if (web_mirror_enabled()) {
        char addr[48];
        if (wifi_net_mode() != 0)
            snprintf(addr, sizeof addr, "WEB %s", wifi_net_ap_ip());
        else if (wifi_net_connected())
            snprintf(addr, sizeof addr, "WEB %s", wifi_net_ip());
        else
            snprintf(addr, sizeof addr, "WEB WAIT WIFI");
        text(6, 34, 1, RGB565(130,230,255), addr);
    } else {
        text(6, 34, 1, RGB565(170,180,200), "WEB OFF");
    }
}

/* ---- spectrum + waterfall ------------------------------------------------ */

static uint16_t heat(float t)
{
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    int r = (int)(255 * (t - 0.5f) * 2.0f), b = (int)(255 * (0.5f - t) * 2.0f);
    int g = (int)(255 * (1.0f - fabsf(t - 0.5f) * 2.0f));
    if (r < 0) r = 0;
    if (b < 0) b = 0;
    if (g < 0) g = 0;
    return RGB565(r, g, b);
}

static TickType_t s_lora_error_until; /* show failed RA-02 activation */
static volatile bool s_ui_force_spectrum;
static int s_panel_mode = -1;      /* which text panel's body is currently drawn (-1 = none/stale) */
static uint32_t s_panel_total = 0; /* message count that body was drawn for */

/* RS41 mode: the waterfall area becomes a telemetry panel. */
static void draw_rs41_panel(void)
{
    static uint32_t last_seq;
    static TickType_t last_tick;
    rs41_info_t in;
    bool have = rs41_get_info(&in);
    s_panel_mode = -1; /* this panel overwrites the shared area */
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0));

    rect(540, WF_Y0 + 3, 92, 23, RGB565(30, 90, 110));
    rs41_smooth_text(555, WF_Y0 + 8, COL_TEXT, "LOG");
    const int x = 8, pitch = 18;
    int y = WF_Y0 + 4;
    char b[128];
    if (!have) {
        rs41_smooth_text(x, y, RGB565(230, 230, 230), "WAITING FOR RS41");
        rs41_smooth_text(x, y + pitch, RGB565(150, 150, 150), "TUNE 400-406 MHZ");
        rs41_smooth_text(x, y + 2 * pitch, RGB565(150, 150, 150), "PRESS RS41 AGAIN TO CENTRE ON THE PEAK");
        return;
    }
    if (in.seq != last_seq) { last_seq = in.seq; last_tick = xTaskGetTickCount(); }
    bool stale = (xTaskGetTickCount() - last_tick) > pdMS_TO_TICKS(4000);
    uint16_t cv = stale ? RGB565(120, 120, 120) : RGB565(230, 230, 230);
    uint16_t ci = stale ? RGB565(120, 120, 120) : RGB565(255, 220, 0);

    snprintf(b, sizeof b, "ID %s  FRM %u", in.have_id ? in.id : "-", (unsigned)in.frame_no);
    rs41_smooth_text(x, y, ci, b);
    y += pitch;
    if (in.have_pos) snprintf(b, sizeof b, "LAT: %.5f  LON: %.5f", in.lat, in.lon);
    else if (in.encrypted) snprintf(b, sizeof b, "POSITION ENCRYPTED  RS41 SGM");
    else snprintf(b, sizeof b, "LAT -  LON -");
    rs41_smooth_text(x, y, cv, b);
    y += pitch;
    if (in.have_pos) snprintf(b, sizeof b, "ALT: %.0f m  VH: %.1f km/h  VV: %.1f m/s  HDG: %.0f", in.alt, (double)(in.vh * 3.6f), in.vv, in.vd);
    else snprintf(b, sizeof b, "ALT -");
    rs41_smooth_text(x, y, cv, b);
    y += pitch;
    if (in.have_time) snprintf(b, sizeof b, "UTC %02d:%02d:%02d  SAT %u", in.hh, in.mm, in.ss, (unsigned)in.sats);
    else snprintf(b, sizeof b, "UTC -");
    rs41_smooth_text(x, y, cv, b);
    y += pitch;
    if (in.rs_fixed < 0) snprintf(b, sizeof b, "OK %u  BAD %u  RS FAIL  MK %u/7  BLK %u", (unsigned)in.frames_ok, (unsigned)in.frames_bad, (unsigned)in.mk, (unsigned)in.blk_ok);
    else snprintf(b, sizeof b, "OK %u  BAD %u  RS %d  MK %u/7  BLK %u", (unsigned)in.frames_ok, (unsigned)in.frames_bad, in.rs_fixed, (unsigned)in.mk, (unsigned)in.blk_ok);
    rs41_smooth_text(x, y, RGB565(150, 150, 150), b);
}

/* FT8 / WSPR: the waterfall area shows the decoded messages, newest first, in three
 * columns of eight lines (24 messages = everything the decoders keep). Each line:
 * audio frequency, SNR, text (cut to fit the column). The header strip is redrawn
 * every frame (it has a running timer); the message body only when the message
 * count changes, so an idle panel costs almost nothing. */
#define PANEL_COLS    3
#define PANEL_ROWS    8
#define PANEL_COL_W   (LW / PANEL_COLS) /* 213 px = 35 characters at scale 1 */
#define PANEL_TEXT_CH 26                /* characters left for the text after "1523 -12 " */
#define PANEL_HDR_H   11

static inline bool is_panel_mode(int m) { return m == DSP_MODE_RS41 || m == DSP_MODE_FT8 || m == DSP_MODE_FT4 || m == DSP_MODE_WSPR; }

/* Prepare the panel: clears what is needed. Returns true if the body must be (re)drawn. */
static bool panel_begin(int mode, uint32_t total)
{
    bool body = (s_panel_mode != mode) || (s_panel_total != total);
    rect(0, WF_Y0, LW, body ? WF_H : PANEL_HDR_H + 2, RGB565(0, 0, 0));
    if (body) { s_panel_mode = mode; s_panel_total = total; }
    return body;
}

static void panel_line(int k, int total_lines, uint16_t color, const char *text_line)
{
    (void)total_lines;
    int col = k / PANEL_ROWS, row = k % PANEL_ROWS;
    text(col * PANEL_COL_W + 4, WF_Y0 + 3 + PANEL_HDR_H + row * 10, 1, color, text_line);
}

static void panel_separators(void)
{
    for (int c = 1; c < PANEL_COLS; c++)
        rect(c * PANEL_COL_W - 3, WF_Y0 + 3 + PANEL_HDR_H - 1, 1, PANEL_ROWS * 10, RGB565(45, 45, 70));
}

/* FT8 QSO: manual and opt-in automatic sequencing. */
static ft8_msg_t s_qso_selected;
static bool s_qso_has_selected = false;
static bool s_cq_active = false;
/* Optional continuous CQ after each completed AUTO QSO. */
static bool s_auto_cq = false;
static int s_cq_tx_parity = 0; /* EVEN UTC 15-second slots */
static int64_t s_cq_last_queued_slot = -1;
static uint32_t s_cq_last_rx_seq = 0;
static char s_cq_debug[76] = "WAIT RX";
static uint32_t s_cq_seen_count = 0;
static bool s_qso_wait_cq = false; /* non-CQ selection: RX monitor until fresh CQ */
static char s_qso_call[16];
static int s_menu;
static char s_qso_preview[48];
static int s_qso_step = 0;
static bool s_qso_error = false;
static const char *s_qso_steps[] = {"CALL", "REPORT", "R-RPT", "RR73", "73"};
/* Automatically arm AUTO SEQ for each newly selected CQ; STOP cancels only current QSO. */
static bool s_auto_enabled = true;
static bool s_auto = false;
static bool s_auto_done = false;
static bool s_auto_logged = false;
/* Final 73 is sent once, then QSO is completed after the entire FT8 frame. */
static bool s_auto_final_queued = false;
static bool s_auto_final_started = false;
static int s_auto_attempts = 0;
static int s_auto_tx_parity = -1;
static bool s_auto_fast_first = false;
static int64_t s_auto_last_queued_slot = -1;
static uint32_t s_auto_last_seq = 0;
static char s_auto_status[80] = "MANUAL";
static char s_qso_rst_sent[8] = "";
static char s_qso_rst_rcvd[8] = "";
static time_t s_qso_start_utc = 0;
static uint64_t s_qso_rf_base_chz = 0;
/* True only when this QSO started with our own CQ: keep TX at 1500 Hz. */
static bool s_qso_own_cq = false;
/* User-adjustable audio offset for our CQ and replies to our own CQ. */
#define CQ_TX_AUDIO_MIN 200
#define CQ_TX_AUDIO_MAX 3000
#define CQ_TX_AUDIO_STEP 50
static uint32_t s_cq_tx_audio_hz = 1500;

static void cq_tx_audio_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ui", NVS_READONLY, &nvs) == ESP_OK) {
        uint32_t value = 0;
        if (nvs_get_u32(nvs, "cqtxhz", &value) == ESP_OK &&
            value >= CQ_TX_AUDIO_MIN && value <= CQ_TX_AUDIO_MAX)
            s_cq_tx_audio_hz = value;
        nvs_close(nvs);
    }
}

static void cq_tx_audio_save(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ui", NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_u32(nvs, "cqtxhz", s_cq_tx_audio_hz) == ESP_OK)
            (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
}

/* FT8 TX monitor volume slider, independent of receiver volume. */
#define TX_MON_X 344
#define TX_MON_W 160
#define TX_MON_Y 20
static int s_tx_mon_vol = 25;
static bool s_tx_mon_drag = false;

static void tx_mon_volume_save(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ui", NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_u8(nvs, "txmonvol", (uint8_t)s_tx_mon_vol) == ESP_OK)
            (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void tx_mon_volume_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ui", NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(nvs, "txmonvol", &v) == ESP_OK && v <= 100)
            s_tx_mon_vol = v;
        nvs_close(nvs);
    }
    audio_ft8_tx_monitor_set_volume(s_tx_mon_vol);
}

static bool tx_mon_volume_touch(int x)
{
    if (x < TX_MON_X) x = TX_MON_X;
    if (x > TX_MON_X + TX_MON_W) x = TX_MON_X + TX_MON_W;
    int vol = ((x - TX_MON_X) * 100 + TX_MON_W / 2) / TX_MON_W;
    if (vol == s_tx_mon_vol) return false;
    s_tx_mon_vol = vol;
    audio_ft8_tx_monitor_set_volume(vol);
    return true;
}

/* Separate QSO transcript: only traffic involving the selected station. */
#define QSO_HISTORY_MAX 7
typedef struct {
    char message[48];
    char stamp[12];
    int snr;
    bool tx;
} qso_history_t;
static qso_history_t s_qso_history[QSO_HISTORY_MAX];
static int s_qso_history_n;
static uint32_t s_qso_history_last_seq;
static bool s_qso_tx_pending;
static bool s_qso_tx_seen;
/* Step captured when a frame is queued; display it from actual RF TX start. */
static int s_qso_tx_step = -1;
static char s_qso_tx_message[48];

static void qso_history_add(const char *message, bool tx, int snr)
{
    if (s_qso_history_n == QSO_HISTORY_MAX) {
        memmove(&s_qso_history[0], &s_qso_history[1],
                sizeof(s_qso_history[0]) * (QSO_HISTORY_MAX - 1));
        s_qso_history_n--;
    }
    qso_history_t *h = &s_qso_history[s_qso_history_n++];
    snprintf(h->message, sizeof h->message, "%s", message);
    time_t now = time(NULL);
    struct tm tm_now;
    if (now >= 1760000000 && gmtime_r(&now, &tm_now))
        strftime(h->stamp, sizeof h->stamp, "%H:%M:%S", &tm_now);
    else snprintf(h->stamp, sizeof h->stamp, "--:--:--");
    h->snr = snr;
    h->tx = tx;
}

static bool qso_history_rx_tick(void)
{
    if ((!s_qso_has_selected && !s_cq_active) ||
        (s_mode != DSP_MODE_FT8 && s_mode != DSP_MODE_FT4)) return false;
    ft8_msg_t msgs[24];
    ft8_status_t st;
    int n = s_mode == DSP_MODE_FT4 ? ft4_get_messages(msgs, 24, &st) : ft8_get_messages(msgs, 24, &st);
    bool changed = false;
    for (int i = 0; i < n; i++) {
        const ft8_msg_t *m = &msgs[i];
        if (m->seq <= s_qso_history_last_seq) continue;
        s_qso_history_last_seq = m->seq;
        char a[20] = "", b[20] = "", c[20] = "";
        if (sscanf(m->text, "%19s %19s %19s", a, b, c) < 2) continue;
        /* Monitor the selected callsign regardless of its QSO partner.
         * The second token is the transmitting station in a directed FT8
         * message; for CQ it is the second token (or third for CQ DX).
         * Keep this independent of AUTO SEQ, including when AUTO is OFF. */
        bool from_station = s_qso_has_selected && ((!strcmp(a, "CQ") &&
                            (!strcmp(b, s_qso_call) || !strcmp(c, s_qso_call))) ||
                            (strcmp(a, "CQ") && !strcmp(b, s_qso_call)));
        if (!from_station) continue;
        qso_history_add(m->text, false, m->snr);
        /* While listening manually, a fresh CQ provides a valid TX slot
         * for a later manual SEND / AUTO ARM. Never transmit on detection. */
        if (s_mode == DSP_MODE_FT8 && s_qso_has_selected && !s_auto && !strcmp(a, "CQ") && m->utc_slot) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            int64_t age = (int64_t)tv.tv_sec / 15 - (int64_t)m->utc_slot;
            if (tv.tv_sec >= 1760000000LL && age >= 0 && age <= 2) {
                s_qso_selected = *m;
                s_qso_wait_cq = false;
                s_auto_tx_parity = (int)((m->utc_slot + 1u) & 1u);
                snprintf(s_auto_status, sizeof s_auto_status,
                         "CQ RECEIVED - MONITOR / READY TO ARM");
            }
        }
        changed = true;
    }
    /* Record a TX only after the Si5351 worker has actually begun the frame. */
    if (s_qso_tx_pending && si5351_tone_test_index() >= 0 && si5351_relay_is_tx()) {
        qso_history_add(s_qso_tx_message, true, 0);
        s_qso_tx_pending = false;
        s_qso_tx_seen = true;
        changed = true;
    }
    if (s_qso_tx_pending && !si5351_tone_test_running())
        s_qso_tx_pending = false; /* aborted before transmission */
    return changed;
}

static void qso_history_queue_tx(void)
{
    snprintf(s_qso_tx_message, sizeof s_qso_tx_message, "%s", s_qso_preview);
    s_qso_tx_pending = true;
    s_qso_tx_seen = false;
    s_qso_tx_step = s_qso_step;
}


static void qso_auto_off(const char *why)
{
    s_auto = false;
    if (why) snprintf(s_auto_status, sizeof s_auto_status, "%s", why);
}

static bool qso_is_report(const char *s)
{
    if (*s == 'R') s++;
    if (*s != '+' && *s != '-') return false;
    return s[1] >= '0' && s[1] <= '9' && s[2] >= '0' && s[2] <= '9' && !s[3];
}

static void ft8_b4_invalidate(void);
static void qso_log_adif(void)
{
    if (s_auto_logged || !s_qso_start_utc) return;
    struct tm tm_start, tm_end;
    time_t now = time(NULL);
    if (now < 1760000000 || !gmtime_r(&s_qso_start_utc, &tm_start) || !gmtime_r(&now, &tm_end)) {
        snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - UTC INVALID, LOG NOT SAVED");
        return;
    }
    if (mkdir("/sdcard/FT8", 0775) != 0 && errno != EEXIST) {
        snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - SD NOT AVAILABLE");
        return;
    }
    FILE *f = fopen("/sdcard/FT8/ft8.adi", "a");
    if (!f) {
        snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - ADIF OPEN FAILED");
        return;
    }
    char d[9], t[7], de[9], te[7], freq[24];
    strftime(d, sizeof d, "%Y%m%d", &tm_start);
    strftime(t, sizeof t, "%H%M%S", &tm_start);
    strftime(de, sizeof de, "%Y%m%d", &tm_end);
    strftime(te, sizeof te, "%H%M%S", &tm_end);
    snprintf(freq, sizeof freq, "%.6f", (double)s_qso_rf_base_chz / 100000000.0);
    fprintf(f, "<CALL:%u>%s <STATION_CALLSIGN:%u>%s <MODE:3>%s "
            "<QSO_DATE:8>%s <TIME_ON:6>%s <QSO_DATE_OFF:8>%s <TIME_OFF:6>%s "
            "<FREQ:%u>%s <RST_SENT:%u>%s <RST_RCVD:%u>%s <EOR>\n",
            (unsigned)strlen(s_qso_call), s_qso_call,
            (unsigned)strlen(station_call()), station_call(),
            s_mode == DSP_MODE_FT4 ? "FT4" : "FT8", d, t, de, te,
            (unsigned)strlen(freq), freq,
            (unsigned)strlen(s_qso_rst_sent), s_qso_rst_sent,
            (unsigned)strlen(s_qso_rst_rcvd), s_qso_rst_rcvd);
    bool ok = !ferror(f);
    if (fclose(f) != 0) ok = false;
    if (ok) {
        s_auto_logged = true;
        ft8_b4_invalidate();
        snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - SAVED /sdcard/FT8/ft8.adi");
    } else snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - ADIF WRITE FAILED");
}

/* FT8 ADIF log viewer: reads the existing SD file without loading it into RAM.
 * Records written by qso_log_adif() occupy one line each. */
#define FT8_LOG_PATH "/sdcard/FT8/ft8.adi"
#define FT8_LOG_TMP  "/sdcard/FT8/ft8.tmp"
#define FT8_LOG_ROWS 8
#define FT8_LOG_LINE 768

typedef struct {
    char call[24], date[12], time_on[12], freq[24], sent[12], rcvd[12], mode[8];
} ft8_log_entry_t;
static ft8_log_entry_t s_ft8_log_rows[FT8_LOG_ROWS];
static int s_ft8_log_count, s_ft8_log_page, s_ft8_log_selected = -1;
static bool s_ft8_log_confirm, s_ft8_log_error;
static char s_ft8_log_notice[72];

static void ft8_log_field(const char *line, const char *key, char *dst, size_t cap)
{
    dst[0] = 0;
    char tag[40];
    snprintf(tag, sizeof tag, "<%s:", key);
    const char *p = line;
    size_t taglen = strlen(tag);
    while ((p = strchr(p, '<')) != NULL) {
        if (!strncasecmp(p, tag, taglen)) {
            const char *n = p + taglen;
            char *end = NULL;
            unsigned long len = strtoul(n, &end, 10);
            if (end == n || !end || (*end != '>' && *end != ':')) return;
            const char *value = strchr(end, '>');
            if (!value) return;
            value++;
            if (len >= cap) len = cap - 1;
            size_t available = strlen(value);
            if (len > available) len = available;
            memcpy(dst, value, len);
            dst[len] = 0;
            return;
        }
        p++;
    }
}

static void ft8_log_parse(const char *line, ft8_log_entry_t *e)
{
    memset(e, 0, sizeof *e);
    ft8_log_field(line, "CALL", e->call, sizeof e->call);
    ft8_log_field(line, "MODE", e->mode, sizeof e->mode);
    ft8_log_field(line, "QSO_DATE", e->date, sizeof e->date);
    ft8_log_field(line, "TIME_ON", e->time_on, sizeof e->time_on);
    ft8_log_field(line, "FREQ", e->freq, sizeof e->freq);
    ft8_log_field(line, "RST_SENT", e->sent, sizeof e->sent);
    ft8_log_field(line, "RST_RCVD", e->rcvd, sizeof e->rcvd);
}

/* Worked-before lookup for FT8 CQ lines. The ADIF file is read only when
 * the index is invalidated, never during decoding or per displayed CQ. */
#define FT8_B4_CAPACITY 4096u
#define FT8_B4_CALL_LEN 20
typedef struct {
    char call[FT8_B4_CALL_LEN];
    uint8_t band;
} ft8_b4_entry_t;
static ft8_b4_entry_t *s_ft8_b4_table;
static bool s_ft8_b4_loaded;
static TickType_t s_ft8_b4_retry_at;

static uint8_t ft8_b4_band(double mhz)
{
    if (mhz >= 1.8 && mhz < 2.0) return 1;
    if (mhz >= 3.5 && mhz < 4.0) return 2;
    if (mhz >= 5.0 && mhz < 5.5) return 3;
    if (mhz >= 7.0 && mhz < 7.3) return 4;
    if (mhz >= 10.0 && mhz < 10.2) return 5;
    if (mhz >= 14.0 && mhz < 14.35) return 6;
    if (mhz >= 18.0 && mhz < 18.2) return 7;
    if (mhz >= 21.0 && mhz < 21.45) return 8;
    if (mhz >= 24.8 && mhz < 25.0) return 9;
    if (mhz >= 28.0 && mhz < 30.0) return 10;
    if (mhz >= 50.0 && mhz < 54.0) return 11;
    if (mhz >= 70.0 && mhz < 71.0) return 12;
    if (mhz >= 144.0 && mhz < 148.0) return 13;
    if (mhz >= 430.0 && mhz < 440.0) return 14;
    return 0;
}

static uint32_t ft8_b4_hash(const char *call, uint8_t band)
{
    uint32_t h = 2166136261u ^ band;
    while (*call) { h = (h ^ (uint8_t)*call++) * 16777619u; }
    return h % FT8_B4_CAPACITY;
}

static bool ft8_b4_find(const char *call, uint8_t band)
{
    if (!s_ft8_b4_loaded || !s_ft8_b4_table || !band) return false;
    uint32_t pos = ft8_b4_hash(call, band);
    for (uint32_t i = 0; i < FT8_B4_CAPACITY; i++) {
        const ft8_b4_entry_t *e = &s_ft8_b4_table[pos];
        if (!e->call[0]) return false;
        if (e->band == band && !strcmp(e->call, call)) return true;
        pos = (pos + 1) % FT8_B4_CAPACITY;
    }
    return false;
}

static void ft8_b4_insert(const char *call, uint8_t band)
{
    if (!s_ft8_b4_table || !band || !*call || strlen(call) >= FT8_B4_CALL_LEN) return;
    uint32_t pos = ft8_b4_hash(call, band);
    for (uint32_t i = 0; i < FT8_B4_CAPACITY; i++) {
        ft8_b4_entry_t *e = &s_ft8_b4_table[pos];
        if (!e->call[0]) {
            snprintf(e->call, sizeof e->call, "%s", call);
            e->band = band;
            return;
        }
        if (e->band == band && !strcmp(e->call, call)) return;
        pos = (pos + 1) % FT8_B4_CAPACITY;
    }
    ESP_LOGW(TAG, "FT8 B4 index full; some worked calls may be unmarked");
}

static void ft8_b4_invalidate(void)
{
    s_ft8_b4_loaded = false;
    s_ft8_b4_retry_at = 0;
    s_panel_mode = -1;
}

static void ft8_b4_ensure_loaded(void)
{
    if (s_ft8_b4_loaded) return;
    TickType_t now = xTaskGetTickCount();
    if (s_ft8_b4_retry_at && (int32_t)(now - s_ft8_b4_retry_at) < 0) return;
    if (!s_ft8_b4_table) {
        s_ft8_b4_table = calloc(FT8_B4_CAPACITY, sizeof *s_ft8_b4_table);
        if (!s_ft8_b4_table) {
            ESP_LOGW(TAG, "FT8 B4 index allocation failed");
            s_ft8_b4_retry_at = now + pdMS_TO_TICKS(10000);
            return;
        }
    }
    FILE *f = fopen(FT8_LOG_PATH, "r");
    if (!f) {
        if (errno == ENOENT) {
            /* No contacts yet: treat every CQ as new. */
            memset(s_ft8_b4_table, 0, FT8_B4_CAPACITY * sizeof *s_ft8_b4_table);
            s_ft8_b4_loaded = true;
            s_panel_mode = -1;
        } else s_ft8_b4_retry_at = now + pdMS_TO_TICKS(10000);
        return;
    }
    memset(s_ft8_b4_table, 0, FT8_B4_CAPACITY * sizeof *s_ft8_b4_table);
    char line[FT8_LOG_LINE], call[FT8_B4_CALL_LEN], freq[24];
    while (fgets(line, sizeof line, f)) {
        if (!strstr(line, "<EOR>") && !strstr(line, "<eor>")) continue;
        ft8_log_field(line, "CALL", call, sizeof call);
        ft8_log_field(line, "FREQ", freq, sizeof freq);
        if (!call[0] || !freq[0]) continue;
        for (char *p = call; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= 32;
        char *freq_end = NULL;
        double mhz = strtod(freq, &freq_end);
        if (freq_end == freq || !isfinite(mhz)) continue;
        while (*freq_end == ' ' || *freq_end == '\t') ++freq_end;
        if (*freq_end) continue;
        ft8_b4_insert(call, ft8_b4_band(mhz));
    }
    bool ok = !ferror(f);
    fclose(f);
    if (!ok) {
        s_ft8_b4_retry_at = now + pdMS_TO_TICKS(10000);
        return;
    }
    s_ft8_b4_loaded = true;
    s_panel_mode = -1;
}

static void ft8_log_refresh(void)
{
    s_ft8_log_count = 0;
    s_ft8_log_error = false;
    FILE *f = fopen(FT8_LOG_PATH, "r");
    if (!f) {
        snprintf(s_ft8_log_notice, sizeof s_ft8_log_notice,
                 errno == ENOENT ? "NO SAVED FT8 QSOS" : "SD / ADIF READ ERROR");
        s_ft8_log_error = errno != ENOENT;
        memset(s_ft8_log_rows, 0, sizeof s_ft8_log_rows);
        return;
    }
    char line[FT8_LOG_LINE];
    while (fgets(line, sizeof line, f)) {
        if (strstr(line, "<EOR>") || strstr(line, "<eor>")) s_ft8_log_count++;
    }
    if (ferror(f)) s_ft8_log_error = true;
    int pages = (s_ft8_log_count + FT8_LOG_ROWS - 1) / FT8_LOG_ROWS;
    if (s_ft8_log_page >= pages) s_ft8_log_page = pages > 0 ? pages - 1 : 0;
    int first = s_ft8_log_count - (s_ft8_log_page + 1) * FT8_LOG_ROWS;
    if (first < 0) first = 0;
    int last = s_ft8_log_count - s_ft8_log_page * FT8_LOG_ROWS;
    memset(s_ft8_log_rows, 0, sizeof s_ft8_log_rows);
    rewind(f);
    int idx = 0;
    while (fgets(line, sizeof line, f)) {
        if (!strstr(line, "<EOR>") && !strstr(line, "<eor>")) continue;
        if (idx >= first && idx < last) {
            /* Display newest record first. */
            ft8_log_parse(line, &s_ft8_log_rows[last - 1 - idx]);
        }
        idx++;
        if (idx >= last) break;
    }
    if (ferror(f)) s_ft8_log_error = true;
    fclose(f);
}

static bool ft8_log_delete(int index)
{
    FILE *in = fopen(FT8_LOG_PATH, "r");
    if (!in) return false;
    FILE *out = fopen(FT8_LOG_TMP, "w");
    if (!out) { fclose(in); return false; }
    char line[FT8_LOG_LINE];
    int idx = 0;
    bool ok = true, found = false;
    while (fgets(line, sizeof line, in)) {
        bool record = strstr(line, "<EOR>") || strstr(line, "<eor>");
        if (record && idx == index) found = true;
        else if (fputs(line, out) == EOF) { ok = false; break; }
        if (record) idx++;
    }
    if (ferror(in)) ok = false;
    if (fclose(in) != 0) ok = false;
    if (fflush(out) != 0) ok = false;
    if (fclose(out) != 0) ok = false;
    if (!ok || !found) { remove(FT8_LOG_TMP); return false; }
    /* FAT rename generally cannot replace an existing target. A backup
     * preserves the original if the new file cannot be installed. */
    const char *backup = "/sdcard/FT8/ft8.bak";
    remove(backup);
    if (rename(FT8_LOG_PATH, backup) != 0) { remove(FT8_LOG_TMP); return false; }
    if (rename(FT8_LOG_TMP, FT8_LOG_PATH) != 0) {
        (void)rename(backup, FT8_LOG_PATH);
        return false;
    }
    remove(backup);
    ft8_b4_invalidate();
    return true;
}

static void ft8_log_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10,14,30));
    text(12, 8, 2, COL_ACCENT, "FT8 / FT4 LOG - SD CARD");
    char b[128];
    snprintf(b, sizeof b, "TOTAL QSOS: %d     PAGE: %d/%d", s_ft8_log_count,
             s_ft8_log_page + 1, (s_ft8_log_count + FT8_LOG_ROWS - 1) / FT8_LOG_ROWS > 0 ?
             (s_ft8_log_count + FT8_LOG_ROWS - 1) / FT8_LOG_ROWS : 1);
    text(12, 38, 1, COL_LINE, b);
    text(12, 63, 1, COL_ACCENT, "UTC DATE   TIME   CALL          MODE   MHz       SENT RCVD");
    rect(10, 81, 620, 193, RGB565(17,25,43));
    int n = s_ft8_log_count - s_ft8_log_page * FT8_LOG_ROWS;
    if (n > FT8_LOG_ROWS) n = FT8_LOG_ROWS;
    for (int i = 0; i < n; ++i) {
        const ft8_log_entry_t *e = &s_ft8_log_rows[i];
        int y = 85 + i * 23;
        if (s_ft8_log_selected == i) rect(12, y - 2, 616, 22, RGB565(38,70,91));
        snprintf(b, sizeof b, "%.8s %.6s %-12.12s %-4.4s %-9.9s %-4.4s %-4.4s",
                 e->date, e->time_on, e->call, e->mode[0] ? e->mode : "?",
                 e->freq, e->sent, e->rcvd);
        text(18, y + 3, 1, COL_TEXT, b);
    }
    if (!s_ft8_log_count) text(22, 115, 2, COL_LINE, "NO LOGGED QSOS YET");
    if (s_ft8_log_error) text(12, 279, 1, RGB565(255,100,100), "SD READ ERROR");
    else if (s_ft8_log_notice[0]) text(12, 279, 1, COL_LINE, s_ft8_log_notice);
    rect(12, 304, 112, 43, COL_BTN); text(29, 319, 2, COL_TEXT, "NEWER");
    rect(134, 304, 112, 43, COL_BTN); text(151, 319, 2, COL_TEXT, "OLDER");
    rect(256, 304, 156, 43, s_ft8_log_confirm ? RGB565(160,48,48) : COL_BTN);
    text(266, 319, 2, COL_TEXT, s_ft8_log_confirm ? "CONFIRM DEL" : "DELETE");
    rect(422, 304, 206, 43, COL_BTN); text(478, 319, 2, COL_TEXT, "BACK");
}

static bool ft8_log_touch(int x, int y)
{
    if (y >= 81 && y < 269 && x >= 10 && x < 630) {
        int i = (y - 85) / 23;
        int n = s_ft8_log_count - s_ft8_log_page * FT8_LOG_ROWS;
        if (n > FT8_LOG_ROWS) n = FT8_LOG_ROWS;
        if (y >= 85 && i >= 0 && i < n) {
            s_ft8_log_selected = i;
            s_ft8_log_confirm = false;
            s_ft8_log_notice[0] = 0;
        }
        return true;
    }
    if (y >= 304 && y < 350) {
        if (x >= 12 && x < 124) {
            if (s_ft8_log_page > 0) s_ft8_log_page--;
            s_ft8_log_selected = -1; s_ft8_log_confirm = false;
            ft8_log_refresh();
        } else if (x >= 134 && x < 246) {
            if ((s_ft8_log_page + 1) * FT8_LOG_ROWS < s_ft8_log_count) s_ft8_log_page++;
            s_ft8_log_selected = -1; s_ft8_log_confirm = false;
            ft8_log_refresh();
        } else if (x >= 256 && x < 412) {
            if (s_ft8_log_selected < 0) {
                snprintf(s_ft8_log_notice, sizeof s_ft8_log_notice, "SELECT A QSO FIRST");
            } else if (!s_ft8_log_confirm) {
                s_ft8_log_confirm = true;
                snprintf(s_ft8_log_notice, sizeof s_ft8_log_notice, "TAP CONFIRM DEL TO DELETE SELECTED QSO");
            } else {
                int index = s_ft8_log_count - 1 - s_ft8_log_page * FT8_LOG_ROWS - s_ft8_log_selected;
                bool ok = ft8_log_delete(index);
                s_ft8_log_selected = -1; s_ft8_log_confirm = false;
                ft8_log_refresh();
                snprintf(s_ft8_log_notice, sizeof s_ft8_log_notice,
                         "%s", ok ? "QSO DELETED" : "DELETE FAILED - CHECK SD");
            }
        } else if (x >= 422 && x < 628) {
            s_menu = 12; s_ft8_log_confirm = false;
        }
    }
    return true;
}

static bool qso_extract_call(const char *message, char *call, size_t cap)
{
    char a[20] = {0}, b[20] = {0}, c[20] = {0};
    int n = sscanf(message, "%19s %19s %19s", a, b, c);
    const char *candidate = NULL;
    if (n >= 2 && strcmp(a, "CQ") == 0) {
        candidate = (strcmp(b, "DX") == 0 && n >= 3) ? c : b;
    } else if (n >= 2) {
        /* Directed FT8 message: TO FROM REPORT/GRID. Monitor the sender,
         * even when the message is addressed to another station. */
        candidate = b;
    }
    if (!candidate || !strpbrk(candidate, "0123456789") ||
        !strpbrk(candidate, "ABCDEFGHIJKLMNOPQRSTUVWXYZ") ||
        strchr(candidate, '<') || strchr(candidate, '>') ||
        strlen(candidate) >= cap) return false;
    snprintf(call, cap, "%s", candidate);
    return true;
}
static void qso_prepare(void)
{
    char extra[16];
    if (s_qso_step == 0) snprintf(extra, sizeof extra, "%s", station_grid());
    else if (s_qso_step == 1) snprintf(extra, sizeof extra, "%+03d", (int)s_qso_selected.snr);
    else if (s_qso_step == 2) snprintf(extra, sizeof extra, "R%+03d", (int)s_qso_selected.snr);
    else if (s_qso_step == 3) snprintf(extra, sizeof extra, "RR73");
    else snprintf(extra, sizeof extra, "73");
    snprintf(s_qso_preview, sizeof s_qso_preview, "%s %s %s", s_qso_call, station_call(), extra);
}
/* Own CQ answered: hold TX at 1500 Hz. Answering someone else: TX follows
 * their decoded audio frequency. RX dial stays unchanged; only retune idle TX. */
static esp_err_t qso_sync_tx_frequency(void)
{
    if (si5351_tone_test_running()) return ESP_ERR_INVALID_STATE;
    if (!s_qso_own_cq && (!s_qso_selected.seq ||
        s_qso_selected.freq < 100 || s_qso_selected.freq > 3500))
        return ESP_ERR_INVALID_ARG;
    uint32_t tx_audio_hz = s_qso_own_cq ? s_cq_tx_audio_hz : (uint32_t)s_qso_selected.freq;
    uint64_t base_chz = ((uint64_t)s_center + tx_audio_hz) * 100ULL;
    esp_err_t err = si5351_ft8_set_base_cHz(base_chz);
    if (err == ESP_OK) s_qso_rf_base_chz = base_chz;
    else ESP_LOGE(TAG, "FT8 TX frequency sync failed: dial %lu Hz, tone %d Hz: %s",
                  (unsigned long)s_center, (int)tx_audio_hz, esp_err_to_name(err));
    return err;
}

/* The previous QSO is replaceable once its final 73 has actually
 * been transmitted and RF is idle.  The AUTO completion tick can lag
 * behind a user's first touch after TX, so do not reopen the old QSO. */
static bool qso_ready_for_new_station(void)
{
    return s_auto_done || (s_qso_has_selected && s_qso_tx_step == 4 &&
           s_qso_tx_seen && !si5351_tone_test_running() &&
           !si5351_relay_is_tx());
}

/* FT8 display filter only: decoding and AUTO SEQ remain unchanged. */
static bool s_ft8_cq_only = false;

static bool ft8_is_cq_message(const char *msg)
{
    return msg && !strncmp(msg, "CQ ", 3);
}

/* Show only messages from the latest decoded FT8 period.  The same filtered
 * array is used for drawing and touch selection, so row mapping stays correct.
 * This does not change the decoder history used by AUTO SEQ. */
static uint32_t s_ft8_visible_period_revision;
static uint32_t s_ft8_visible_period_last_total = UINT32_MAX;
static uint32_t s_ft8_visible_period_last_utc;
static uint16_t s_ft8_visible_period_last_slot;
static int s_ft8_visible_period_last_count = -1;

static int ft8_visible_messages(ft8_msg_t *out, int capacity, ft8_status_t *st)
{
    ft8_msg_t all[PANEL_COLS * PANEL_ROWS];
    int n = s_mode == DSP_MODE_FT4 ? ft4_get_messages(all, PANEL_COLS * PANEL_ROWS, st) :
            ft8_get_messages(all, PANEL_COLS * PANEL_ROWS, st);
    int count = 0;
    uint32_t latest_utc = n > 0 ? all[n - 1].utc_slot : 0;
    uint16_t latest_slot = n > 0 ? all[n - 1].slot : 0;
    for (int i = 0; i < n; ++i) {
        /* utc_slot is authoritative when available; slot is the fallback
         * for early decodes before the UTC clock has synchronized. */
        /* FT4 can have a mixture of UTC-tagged and untagged messages.
         * Use its local decoded slot consistently, never mix domains. */
        bool same_period = (s_mode == DSP_MODE_FT4)
            ? (all[i].slot == latest_slot)
            : (latest_utc && all[i].utc_slot
                ? all[i].utc_slot == latest_utc
                : all[i].slot == latest_slot);
        if (!same_period) continue;
        if (s_ft8_cq_only && !ft8_is_cq_message(all[i].text)) continue;
        if (count < capacity) out[count++] = all[i];
    }
    /* CQ ONLY: sort by SNR ascending internally, because both the FT8
     * panel and touch selection read this array in reverse order.
     * Strongest CQ therefore appears first. Equal SNR keeps decode order.
     * ALL mode retains its original order. */
    if (s_ft8_cq_only) {
        for (int i = 1; i < count; ++i) {
            ft8_msg_t item = out[i];
            int j = i;
            while (j > 0 && out[j - 1].snr > item.snr) {
                out[j] = out[j - 1];
                --j;
            }
            out[j] = item;
        }
    }
    if (st->total != s_ft8_visible_period_last_total ||
        latest_utc != s_ft8_visible_period_last_utc ||
        latest_slot != s_ft8_visible_period_last_slot ||
        count != s_ft8_visible_period_last_count) {
        s_ft8_visible_period_last_total = st->total;
        s_ft8_visible_period_last_utc = latest_utc;
        s_ft8_visible_period_last_slot = latest_slot;
        s_ft8_visible_period_last_count = count;
        ++s_ft8_visible_period_revision;
    }
    return count;
}

static bool qso_select_panel_row(int x, int y)
{
    /* Wider hit target horizontally; preserve 8 independently selectable rows. */
    if (y < WF_Y0 + 3 + PANEL_HDR_H || y >= WF_Y0 + 3 + PANEL_HDR_H + PANEL_ROWS * 10) return false;
    int col = x / PANEL_COL_W;
    int row = (y - (WF_Y0 + 3 + PANEL_HDR_H)) / 10;
    if (col < 0 || col >= PANEL_COLS || row < 0 || row >= PANEL_ROWS) return false;
    int k = col * PANEL_ROWS + row;
    ft8_msg_t msgs[PANEL_COLS * PANEL_ROWS];
    ft8_status_t st;
    int n = ft8_visible_messages(msgs, PANEL_COLS * PANEL_ROWS, &st);
    if (k >= n) return false;
    const ft8_msg_t *m = &msgs[n - 1 - k];
    char call[sizeof s_qso_call];
    if (!qso_extract_call(m->text, call, sizeof call)) return false;
    /* Tapping the same station while a QSO is in progress must reopen the
     * existing QSO, not reset its AUTO SEQ state. */
    if (s_qso_has_selected && !qso_ready_for_new_station() &&
        !strcmp(call, s_qso_call)) return true;
    /* Never replace an active automatic QSO by tapping another station. */
    if (s_qso_has_selected && !qso_ready_for_new_station() &&
        (s_auto || si5351_tone_test_running()))
        return false;
    if (s_cq_active && si5351_tone_test_running()) return false;
    s_cq_active = false; /* Selecting a station stops repeated CQ calls. */
    s_qso_own_cq = false; /* Answering another CQ follows their audio frequency. */
    s_qso_selected = *m;
    if (s_mode == DSP_MODE_FT4) ft4_set_qso_focus(call, (int)m->freq);
    else ft8_set_qso_focus(call, (int)m->freq);
    snprintf(s_qso_call, sizeof s_qso_call, "%s", call);
    s_qso_history_n = 0;
    s_qso_history_last_seq = m->seq;
    s_qso_tx_pending = false;
    s_qso_tx_seen = false;
    s_qso_tx_step = -1;
    qso_history_add(m->text, false, m->snr);
    s_qso_step = 0;
    s_qso_error = false;
    /* A non-CQ message is for monitoring only. Never auto-transmit into
     * somebody else's QSO, even if AUTO SEQ was previously enabled. */
    bool selected_cq = !strncmp(m->text, "CQ ", 3);
    s_qso_wait_cq = !selected_cq;
    /* Selecting a fresh FT4 CQ immediately arms AUTO SEQ, just like FT8.
     * Non-CQ rows remain monitor-only. The FT4 worker still verifies UTC
     * parity and queues only in the preceding RX slot. */
    s_auto = selected_cq && s_auto_enabled &&
             (s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4);
    s_auto_done = false;
    s_auto_logged = false;
    s_auto_final_queued = false;
    s_auto_final_started = false;
    s_auto_attempts = 0;
    /* Determine opposite TX parity from the actual UTC slot of this decoded
     * message. Never infer UTC parity from decoder counters: scan/re-lock and
     * early focused decodes do not have a fixed publication delay. */
    struct timeval sel_tv;
    gettimeofday(&sel_tv, NULL);
    int64_t now_slot = ((int64_t)sel_tv.tv_sec * 1000000LL + sel_tv.tv_usec) /
                       (s_mode == DSP_MODE_FT4 ? 7500000LL : 15000000LL);
    int64_t rx_age = now_slot - (int64_t)m->utc_slot;
    if ((s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) &&
        sel_tv.tv_sec >= 1760000000LL && m->utc_slot != 0 &&
        rx_age >= 0 && rx_age <= 2) {
        s_auto_tx_parity = (int)((m->utc_slot + 1u) & 1u);
        ESP_LOGI(TAG, "QSO %s: RX UTC slot %lu, age %lld, TX parity %d",
                 call, (unsigned long)m->utc_slot, (long long)rx_age,
                 s_auto_tx_parity);
    } else {
        s_auto_tx_parity = -1;
        ESP_LOGW(TAG, "QSO %s: RX UTC slot not verified (slot=%lu, age=%lld)",
                 call, (unsigned long)m->utc_slot, (long long)rx_age);
    }
    s_auto_last_queued_slot = -1;
    if (s_mode == DSP_MODE_FT4 && s_auto && s_auto_tx_parity < 0) {
        /* Do not transmit without a verified opposite slot. */
        s_auto = false;
    }
    s_auto_fast_first = selected_cq && s_auto && s_auto_tx_parity >= 0;
    s_auto_last_seq = m->seq;
    s_qso_rst_sent[0] = s_qso_rst_rcvd[0] = 0;
    s_qso_start_utc = 0;
    /* On a fresh CQ, arm RF frequency immediately; monitoring stays passive. */
    s_qso_error = s_auto && (qso_sync_tx_frequency() != ESP_OK);
    snprintf(s_auto_status, sizeof s_auto_status, "%s",
             s_auto ? (s_auto_tx_parity < 0 ? "AUTO WAIT - SELECT FRESH CQ" : "AUTO ARMED - WAIT TX SLOT") :
             (selected_cq ? "CQ SELECTED - AUTO OFF" : "MONITORING - WAIT FOR CQ"));
    qso_prepare();
    s_qso_has_selected = true;
    return true;
}
/* Called from the UI task even while the QSO menu is open. No transmission
 * starts unless AUTO was explicitly enabled. Repeats only on the same
 * 30-second parity; max 3 attempts for each message. */
/* FT4 automatic QSO: schedule only the opposite 7.5 s parity.
 * The FT4 Si5351 worker intentionally has no late-start path. Queue in the
 * preceding RX slot, never inside the TX slot, to prevent crossed periods. */
static bool ft4_auto_tick(void)
{
    if (!s_auto || !s_qso_has_selected || s_auto_done || s_mode != DSP_MODE_FT4) return false;
    if (s_auto_final_queued) {
        if (si5351_relay_is_tx() && si5351_tone_test_index() >= 0)
            s_auto_final_started = true;
        if (s_auto_final_started && !si5351_tone_test_running()) {
            s_auto_final_queued = false;
            s_auto_done = true;
            s_auto = false;
            qso_log_adif();
            if (s_auto_cq) {
                s_cq_active = true;
                s_qso_has_selected = false;
                s_qso_own_cq = false;
                s_qso_call[0] = 0;
                s_cq_last_queued_slot = -1;
                ft8_msg_t cq_msgs[24];
                ft8_status_t cq_st;
                int cq_n = ft4_get_messages(cq_msgs, 24, &cq_st);
                s_cq_last_rx_seq = cq_n ? cq_msgs[cq_n - 1].seq : 0;
                ft4_set_qso_focus(NULL, 0);
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 QSO LOGGED - CQ RESUMED");
            }
            return true;
        }
        if (!s_auto_final_started && !si5351_tone_test_running()) {
            s_auto_final_queued = false;
            s_qso_error = true;
            qso_auto_off("FT4 FINAL TX ABORTED");
            return true;
        }
        return false;
    }
    bool changed = false;
    ft8_msg_t msgs[24];
    ft8_status_t st;
    int n = ft4_get_messages(msgs, 24, &st);
    for (int i = 0; i < n; ++i) {
        const ft8_msg_t *m = &msgs[i];
        if (m->seq <= s_auto_last_seq) continue;
        s_auto_last_seq = m->seq;
        char to[20] = "", from[20] = "", extra[20] = "";
        if (sscanf(m->text, "%19s %19s %19s", to, from, extra) != 3) continue;
        if (strcmp(to, station_call()) || strcmp(from, s_qso_call)) continue;
        ft4_set_qso_focus(s_qso_call, (int)m->freq);
        s_qso_selected = *m; /* refresh SNR and tracked TX audio offset */
        changed = true;
        if (qso_is_report(extra)) {
            snprintf(s_qso_rst_rcvd, sizeof s_qso_rst_rcvd, "%.7s", extra[0] == 'R' ? extra + 1 : extra);
            s_qso_step = extra[0] == 'R' ? 3 : 2;
            s_auto_attempts = 0;
            snprintf(s_auto_status, sizeof s_auto_status, "FT4 RX %s - NEXT %s", extra, s_qso_steps[s_qso_step]);
        } else if (!strcmp(extra, "RR73") || !strcmp(extra, "73")) {
            s_qso_step = 4;
            s_auto_attempts = 0;
            snprintf(s_auto_status, sizeof s_auto_status, "FT4 RX %s - FINAL 73", extra);
        }
    }
    if (changed) qso_prepare();
    if (si5351_tone_test_running() || s_auto_tx_parity < 0) return changed;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec < 1760000000LL) return changed;
    const int64_t utc_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
    const int64_t slot = utc_us / 7500000LL;
    const int64_t elapsed = utc_us % 7500000LL;
    /* Queue as soon as the CQ is selected and AUTO SEQ is armed.
     * Waiting until the last 1.2 s of RX can miss the next TX slot
     * when the decode/UI arrives late. The Si5351 worker still starts
     * only on the correct UTC slot parity. Avoid the last 0.5 s,
     * where its scheduling guard intentionally advances a slot. */
    if (((slot + 1) & 1LL) != s_auto_tx_parity ||
        elapsed > 6900000LL) return changed;
    const int64_t tx_slot = slot + 1;
    if (s_auto_last_queued_slot == tx_slot) return changed;
    if (s_auto_attempts >= 5) {
        qso_auto_off("FT4 AUTO STOP - NO REPLY AFTER 5 TX");
        return true;
    }
    char to[16], from[16], extra[16];
    if (sscanf(s_qso_preview, "%15s %15s %15s", to, from, extra) != 3) {
        qso_auto_off("FT4 AUTO - INVALID MESSAGE"); return true;
    }
    esp_err_t err = qso_sync_tx_frequency();
    if (err == ESP_OK) err = si5351_ft4_set_message(to, from, extra);
    if (err == ESP_OK) err = si5351_ft4_start_parity(s_auto_tx_parity);
    if (err != ESP_OK) {
        s_qso_error = true;
        qso_auto_off("FT4 AUTO - TX ERROR");
        return true;
    }
    qso_history_queue_tx();
    if (s_qso_step == 4) {
        s_auto_final_queued = true;
        s_auto_final_started = false;
    }
    if (!s_qso_start_utc) s_qso_start_utc = (time_t)(tx_slot * 7500000LL / 1000000LL);
    if (s_qso_step == 2) snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", extra + 1);
    else if (s_qso_step == 1) snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", extra);
    s_auto_last_queued_slot = tx_slot;
    s_auto_attempts++;
    snprintf(s_auto_status, sizeof s_auto_status, "FT4 AUTO %s %d/5", s_qso_steps[s_qso_step], s_auto_attempts);
    return true;
}

static bool qso_auto_tick(void)
{
    if (!s_auto || !s_qso_has_selected || s_auto_done || s_mode != DSP_MODE_FT8) return false;
    bool changed = false;
    /* Once the final 73 was queued, wait for the real transmission to start
     * and finish. Never mark the QSO complete while TX is still running. */
    if (s_auto_final_queued) {
        if (si5351_relay_is_tx() && si5351_tone_test_index() >= 0)
            s_auto_final_started = true;
        if (s_auto_final_started && !si5351_tone_test_running()) {
            s_auto_final_queued = false;
            s_auto_done = true;
            s_auto = false;
            qso_log_adif();
            if (s_auto_cq) {
                /* RF is idle: resume normal CQ scheduling on the original
                 * parity. The CQ worker will wait for the next valid slot. */
                s_cq_active = true;
                s_qso_own_cq = false;
                s_cq_last_queued_slot = -1;
                s_cq_seen_count = 0;
                s_cq_last_rx_seq = s_auto_last_seq;
                s_qso_has_selected = false;
                s_qso_call[0] = 0;
                s_qso_history_n = 0;
                s_qso_history_last_seq = 0;
                s_qso_tx_pending = false;
                s_qso_tx_step = -1;
                s_auto_fast_first = false;
                ft8_set_qso_focus(NULL, 0);
                snprintf(s_cq_debug, sizeof s_cq_debug, "AUTO QSO - WAIT NEXT TX SLOT");
                snprintf(s_auto_status, sizeof s_auto_status, "QSO DONE - AUTO QSO RESUMED");
            }
            return true;
        }
        if (!s_auto_final_started && !si5351_tone_test_running()) {
            s_auto_final_queued = false;
            s_qso_error = true;
            qso_auto_off("FINAL 73 ABORTED - SEND MANUALLY");
            return true;
        }
        return false;
    }
    ft8_msg_t msgs[24];
    ft8_status_t st;
    int n = ft8_get_messages(msgs, 24, &st);
    for (int i = 0; i < n; ++i) {
        const ft8_msg_t *m = &msgs[i];
        if (m->seq <= s_auto_last_seq) continue;
        s_auto_last_seq = m->seq;
        char to[20] = "", from[20] = "", extra[20] = "";
        if (sscanf(m->text, "%19s %19s %19s", to, from, extra) != 3) continue;
        if (strcmp(to, station_call()) || strcmp(from, s_qso_call)) {
            continue;
        }
        ft8_set_qso_focus(s_qso_call, (int)m->freq);
        changed = true;
        if (qso_is_report(extra)) {
            snprintf(s_qso_rst_rcvd, sizeof s_qso_rst_rcvd, "%.7s", extra[0] == 'R' ? extra + 1 : extra);
            if (extra[0] == 'R') {
                s_qso_step = 3; /* received R-report: reply RR73 */
            } else {
                s_qso_step = 2; /* received report: reply R-report */
            }
            s_auto_attempts = 0;
            snprintf(s_auto_status, sizeof s_auto_status, "RX %s - NEXT %s", extra, s_qso_steps[s_qso_step]);
        } else if (!strcmp(extra, "RR73") || !strcmp(extra, "73")) {
            /* Acknowledgement received: we still owe the other station 73.
             * Do not stop a frame that is already transmitting. */
            if (s_qso_step != 4) {
                s_qso_step = 4;
                s_auto_attempts = 0;
                snprintf(s_auto_status, sizeof s_auto_status, "RX %s - FINAL 73 NEXT", extra);
            }
        } else if (s_qso_step == 0) {
            /* A grid reply is not a report; wait for a signal report. */
            snprintf(s_auto_status, sizeof s_auto_status, "RX GRID - WAIT REPORT");
        }
    }
    /* Rebuild the actual TX message after a decoded reply changes the
     * QSO step.  Without this, the UI highlights RR73/R-RPT while the
     * Si5351 encoder keeps transmitting the previous REPORT packet. */
    if (changed) qso_prepare();
    if (!s_auto) return changed;
    if (si5351_tone_test_running()) return changed;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec < 1760000000) {
        snprintf(s_auto_status, sizeof s_auto_status, "AUTO WAITING FOR UTC/NTP");
        return changed;
    }
    int64_t slot = (int64_t)tv.tv_sec / 15;
    int sec_in_slot = (int)(tv.tv_sec % 15);
    /* Queue as soon as possible; the worker enforces the correct UTC parity.
     * Avoid the last second to allow the I2C and relay to settle. */
    if (s_auto_tx_parity < 0) {
        snprintf(s_auto_status, sizeof s_auto_status,
                 "AUTO WAIT: TX SLOT UNKNOWN - SELECT FRESH CQ");
        return changed;
    }
    /* All AUTO QSO steps may use the current parity-matched TX slot
     * when decoding finishes just after its boundary. */
    /* FAST AUTO SEQ for every QSO step, not just the first CQ reply.
     * The Si5351 worker enforces the actual <=1 s late-start limit.
     * Queue only if there is still time to encode and prepare the relay. */
    /* FT8 decoding completes near the END of the peer's RX slot.
     * Never queue a repeat early in that RX slot: the previous message
     * may still be awaiting its decoded reply. Instead queue at the
     * beginning of OUR TX slot, after the decoder had the whole RX slot.
     * Si5351 allows a safe <=1 s fast start on the current parity. */
    /* A CQ may already be on air when a delayed decoder result arrives.
     * After cq_tick() stops that CQ, allow its replacement QSO message
     * in the SAME TX slot, provided we are still within the 1 s window.
     * The Si5351 worker makes the final timing decision. */
    bool fast_first = (slot & 1LL) == s_auto_tx_parity &&
                      sec_in_slot == 0 && tv.tv_usec < 1000000;
    if (!fast_first) return changed;
    int64_t next_tx_slot = slot;
    if (s_auto_last_queued_slot == next_tx_slot) return changed;
    if (s_auto_attempts >= 5) {
        /* No reply after the configured number of attempts. If AUTO QSO is
         * enabled, abandon only this QSO and resume CQ on the original
         * CQ parity. Do not turn off the AUTO QSO user preference. */
        if (s_auto_cq) {
            s_auto = false;
            s_auto_done = false;
            s_auto_final_queued = false;
            s_auto_final_started = false;
            s_auto_attempts = 0;
            s_auto_fast_first = false;
            s_auto_last_queued_slot = -1;
            s_qso_has_selected = false;
            s_qso_wait_cq = false;
            s_qso_own_cq = false;
            s_qso_call[0] = 0;
            s_qso_history_n = 0;
            s_qso_history_last_seq = 0;
            s_qso_tx_pending = false;
            s_qso_tx_seen = false;
            s_qso_tx_step = -1;
            s_qso_error = false;
            ft8_set_qso_focus(NULL, 0);
            s_cq_active = true;
            s_cq_last_queued_slot = -1;
            s_cq_seen_count = 0;
            s_cq_last_rx_seq = s_auto_last_seq;
            snprintf(s_cq_debug, sizeof s_cq_debug,
                     "AUTO QSO - QSO TIMEOUT, WAIT NEXT TX SLOT");
            snprintf(s_auto_status, sizeof s_auto_status,
                     "NO RESPONSE AFTER 5 TX - AUTO QSO RESUMED");
        } else {
            qso_auto_off("AUTO STOP: NO RESPONSE AFTER 5 TX");
        }
        return true;
    }
    char to[16], from[16], extra[16];
    if (sscanf(s_qso_preview, "%15s %15s %15s", to, from, extra) != 3) {
        s_qso_error = true; qso_auto_off("AUTO STOP: INVALID MESSAGE"); return true;
    }
    esp_err_t err = qso_sync_tx_frequency();
    if (err == ESP_OK) err = si5351_ft8_set_message(to, from, extra);
    if (err == ESP_OK) err = si5351_tone_test_start_parity(s_auto_tx_parity);
    if (err == ESP_OK) {
        qso_history_queue_tx();
        if (s_qso_step == 4) {
            s_auto_final_queued = true;
            s_auto_final_started = false;
        }
    }
    if (err != ESP_OK) {
        s_qso_error = true; qso_auto_off("AUTO STOP: TX ERROR"); return true;
    }
    if (!s_qso_start_utc) s_qso_start_utc = (time_t)(next_tx_slot * 15);
    if (s_qso_step == 2) snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", extra + 1);
    else if (s_qso_step == 1) snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", extra);
    s_auto_last_queued_slot = next_tx_slot;
    s_auto_fast_first = false;
    s_auto_attempts++;
    snprintf(s_auto_status, sizeof s_auto_status, "%s %s %d/5",
             fast_first ? "FAST TX" : "AUTO TX", s_qso_steps[s_qso_step], s_auto_attempts);
    return true;
}

/* CQ calls use the existing FT8 encoder and TX worker. The first fresh,
 * directed reply from the opposite RX slot starts an automatic QSO. */
/* FT4 AUTO CQ: receive on one 7.5 s parity, transmit on the other.
 * Never start late: the same pre-slot scheduling window as ft4_auto_tick(). */
static bool ft4_cq_tick(void)
{
    if (!s_cq_active || s_mode != DSP_MODE_FT4) return false;
    ft8_msg_t msgs[24];
    ft8_status_t st;
    int n = ft4_get_messages(msgs, 24, &st);
    for (int i = 0; i < n; ++i) {
        const ft8_msg_t *m = &msgs[i];
        if (m->seq <= s_cq_last_rx_seq) continue;
        s_cq_last_rx_seq = m->seq;
        char to[20] = "", from[20] = "", extra[20] = "";
        if (sscanf(m->text, "%19s %19s %19s", to, from, extra) != 3) continue;
        if (strcmp(to, station_call()) || !strcmp(from, station_call()) ||
            !strcmp(from, "CQ") || s_cq_last_queued_slot < 0) continue;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        if (tv.tv_sec < 1760000000LL || !m->utc_slot) continue;
        int64_t now_slot = ((int64_t)tv.tv_sec * 1000000LL + tv.tv_usec) / 7500000LL;
        if (now_slot < (int64_t)m->utc_slot || now_slot - (int64_t)m->utc_slot > 2 ||
            ((int64_t)m->utc_slot & 1LL) == s_cq_tx_parity) continue;
        char call[sizeof s_qso_call];
        if (!qso_extract_call(m->text, call, sizeof call) || strcmp(call, from)) continue;
        /* A CQ frame might already be queued. Cancel it before handing
         * scheduling over to FT4 AUTO SEQ; never transmit both frames. */
        if (si5351_tone_test_running()) si5351_tone_test_stop();
        s_cq_active = false;
        s_qso_own_cq = true;
        s_qso_selected = *m;
        snprintf(s_qso_call, sizeof s_qso_call, "%s", call);
        s_qso_has_selected = true;
        s_qso_wait_cq = false;
        s_qso_history_n = 0;
        s_qso_history_last_seq = m->seq;
        s_qso_tx_pending = false;
        s_qso_tx_seen = false;
        s_qso_tx_step = -1;
        qso_history_add(m->text, false, m->snr);
        s_qso_step = qso_is_report(extra) ? 2 : 1; /* grid -> REPORT, report -> R-RPT */
        s_qso_error = false;
        s_auto = true;
        s_auto_enabled = true;
        s_auto_done = false;
        s_auto_logged = false;
        s_auto_final_queued = false;
        s_auto_final_started = false;
        s_auto_attempts = 0;
        s_auto_tx_parity = s_cq_tx_parity;
        s_auto_last_queued_slot = -1;
        s_auto_last_seq = m->seq;
        s_qso_rst_sent[0] = s_qso_rst_rcvd[0] = 0;
        if (qso_is_report(extra))
            snprintf(s_qso_rst_rcvd, sizeof s_qso_rst_rcvd, "%.7s",
                     extra[0] == 'R' ? extra + 1 : extra);
        s_qso_start_utc = 0;
        ft4_set_qso_focus(s_qso_call, (int)m->freq);
        qso_prepare();
        snprintf(s_auto_status, sizeof s_auto_status,
                 "FT4 CQ ANSWER %.15s - AUTO REPORT", s_qso_call);
        return true;
    }
    if (si5351_tone_test_running() || si5351_relay_is_tx()) return false;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec < 1760000000LL) return false;
    int64_t utc_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
    int64_t slot = utc_us / 7500000LL;
    int64_t elapsed = utc_us % 7500000LL;
    if (((slot + 1) & 1LL) != s_cq_tx_parity ||
        elapsed < 6200000LL || elapsed > 7000000LL) return false;
    int64_t tx_slot = slot + 1;
    if (s_cq_last_queued_slot == tx_slot) return false;
    esp_err_t err = si5351_ft8_set_base_cHz(
        ((uint64_t)s_center + s_cq_tx_audio_hz) * 100ULL);
    if (err == ESP_OK) err = si5351_ft4_set_message("CQ", station_call(), station_grid());
    if (err == ESP_OK) err = si5351_ft4_start_parity(s_cq_tx_parity);
    if (err != ESP_OK) {
        s_cq_active = false;
        s_qso_error = true;
        snprintf(s_auto_status, sizeof s_auto_status,
                 "FT4 CQ TX ERROR: %s", esp_err_to_name(err));
        return true;
    }
    s_cq_last_queued_slot = tx_slot;
    s_qso_tx_step = -1;
    snprintf(s_qso_tx_message, sizeof s_qso_tx_message,
             "CQ %s %s", station_call(), station_grid());
    s_qso_tx_pending = true;
    s_qso_tx_seen = false;
    snprintf(s_auto_status, sizeof s_auto_status,
             "FT4 CQ TX QUEUED - %lu HZ", (unsigned long)s_cq_tx_audio_hz);
    return true;
}

static bool cq_tick(void)
{
    if (!s_cq_active || s_mode != DSP_MODE_FT8) return false;
    bool changed = false;
    static bool reply_pending = false;
    static ft8_msg_t pending_reply;
    ft8_msg_t msgs[24];
    ft8_status_t st;
    int n = ft8_get_messages(msgs, 24, &st);
    for (int i = 0; i < n && !reply_pending; ++i) {
        const ft8_msg_t *m = &msgs[i];
        if (m->seq <= s_cq_last_rx_seq) continue;
        s_cq_last_rx_seq = m->seq;
        s_cq_seen_count++;
        snprintf(s_cq_debug, sizeof s_cq_debug, "RX %lu: %.52s", (unsigned long)s_cq_seen_count, m->text);
        /* A directed FT8 reply is TO FROM [GRID/REPORT].
         * Accept the first NEW decoded message addressed to our callsign.
         * Do not require a particular third field or a validated utc_slot:
         * FT8 decoder publication can lag and utc_slot may be zero. */
        char to[20] = "", from[20] = "", extra[20] = "";
        if (sscanf(m->text, "%19s %19s %19s", to, from, extra) < 2) continue;
        if (strcmp(to, station_call()) != 0 || strcmp(from, station_call()) == 0 ||
            strcmp(from, "CQ") == 0) continue;
        if (s_cq_last_queued_slot < 0) continue; /* No CQ queued yet. */
        char call[sizeof s_qso_call];
        if (!qso_extract_call(m->text, call, sizeof call) || strcmp(call, from)) continue;
        /* Latch the reply even if a CQ TX is queued or active. */
        snprintf(s_cq_debug, sizeof s_cq_debug, "MATCH: %.54s", m->text);
        pending_reply = *m;
        reply_pending = true;
        /* Keep the current CQ TX running: replace remaining symbols with
         * the QSO reply, retaining the original UTC symbol timeline. */

    }
    if (reply_pending) {
        const ft8_msg_t *m = &pending_reply;
        char call[sizeof s_qso_call];
        if (!qso_extract_call(m->text, call, sizeof call)) {
            reply_pending = false;
            return true;
        }
        char to[20] = "", from[20] = "", extra[20] = "";
        (void)sscanf(m->text, "%19s %19s %19s", to, from, extra);
        reply_pending = false;
        /* The first qualifying decoded reply wins; later stations are ignored
         * until this QSO is finished or manually stopped. */
        s_cq_active = false;
        s_qso_own_cq = true; /* Our CQ was answered: TX stays at 1500 Hz. */
        snprintf(s_cq_debug, sizeof s_cq_debug, "QSO START: %.50s", m->text);
        /* CQ MATCH must initialize the selected FT8 message as well.
         * qso_sync_tx_frequency() requires its seq and audio frequency. */
        s_qso_selected = *m;
        snprintf(s_qso_call, sizeof s_qso_call, "%s", call);
        s_qso_has_selected = true;
        s_qso_wait_cq = false;
        s_qso_history_n = 0;
        s_qso_history_last_seq = m->seq;
        s_qso_tx_pending = false;
        s_qso_tx_seen = false;
        s_qso_tx_step = -1;
        qso_history_add(m->text, false, m->snr);
        s_qso_step = qso_is_report(extra) ? 2 : 1;
        /* A reply containing a report requires an R-report; a grid requires a report. */
        s_qso_error = false;
        s_auto = true;
        s_auto_enabled = true;
        s_auto_done = false;
        s_auto_logged = false;
        s_auto_final_queued = false;
        s_auto_final_started = false;
        s_auto_attempts = 0;
        s_auto_tx_parity = s_cq_tx_parity;
        /* CQ reply decoded at the RX/TX boundary: permit first REPORT
         * in the current TX slot within the Si5351 safe late window. */
        s_auto_fast_first = true;
        s_auto_last_queued_slot = -1;
        s_auto_last_seq = m->seq;
        s_qso_rst_sent[0] = 0;
        s_qso_rst_rcvd[0] = 0;
        if (qso_is_report(extra))
            snprintf(s_qso_rst_rcvd, sizeof s_qso_rst_rcvd, "%.7s", extra[0] == 'R' ? extra + 1 : extra);
        s_qso_start_utc = 0;
        ft8_set_qso_focus(s_qso_call, (int)m->freq);
        qso_prepare();
        /* The CQ worker may already be on air. Switch its encoded tones at
         * the next symbol boundary, without dropping relay or restarting.
         * The existing frame ends at its original UTC time. */
        if (si5351_tone_test_running()) {
            char tx_to[16], tx_from[16], tx_extra[16];
            if (sscanf(s_qso_preview, "%15s %15s %15s", tx_to, tx_from, tx_extra) == 3 &&
                si5351_ft8_replace_active_message(tx_to, tx_from, tx_extra) == ESP_OK) {
                struct timeval sw_tv;
                gettimeofday(&sw_tv, NULL);
                s_auto_last_queued_slot = (int64_t)sw_tv.tv_sec / 15;
                s_auto_attempts = 1;
                qso_history_queue_tx();
                if (!s_qso_start_utc) s_qso_start_utc = sw_tv.tv_sec;
                if (s_qso_step == 2)
                    snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", tx_extra + 1);
                else if (s_qso_step == 1)
                    snprintf(s_qso_rst_sent, sizeof s_qso_rst_sent, "%.7s", tx_extra);
                snprintf(s_auto_status, sizeof s_auto_status,
                         "CQ REPLY %.15s - SWITCH TX NOW", s_qso_call);
                return true;
            }
        }
        snprintf(s_auto_status, sizeof s_auto_status,
                 "CQ ANSWER %.15s - AUTO REPORT NEXT", s_qso_call);
        return true;
    }
    if (si5351_tone_test_running()) return changed;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec < 1760000000LL) {
        snprintf(s_auto_status, sizeof s_auto_status, "CQ WAIT - UTC NOT VALID");
        return false;
    }
    int64_t slot = (int64_t)tv.tv_sec / 15;
    int sec = (int)(tv.tv_sec % 15);
    if (((slot + 1) & 1LL) != s_cq_tx_parity) return changed;
    if (sec < 13 || (sec == 13 && tv.tv_usec < 650000)) return changed;
    if (sec >= 14 && tv.tv_usec >= 700000) return changed;
    int64_t tx_slot = slot + 1;
    if (s_cq_last_queued_slot == tx_slot) return changed;
    /* 1500 Hz audio tone; keep the receiver dial frequency unchanged. */
    uint64_t base = ((uint64_t)s_center + (uint64_t)s_cq_tx_audio_hz) * 100ULL;
    esp_err_t err = si5351_ft8_set_base_cHz(base);
    if (err == ESP_OK) err = si5351_ft8_set_message("CQ", station_call(), station_grid());
    if (err == ESP_OK) err = si5351_tone_test_start_parity(s_cq_tx_parity);
    if (err != ESP_OK) {
        s_cq_active = false;
        s_qso_error = true;
        snprintf(s_auto_status, sizeof s_auto_status, "CQ TX ERROR: %s", esp_err_to_name(err));
        return true;
    }
    s_cq_last_queued_slot = tx_slot;
    s_qso_tx_step = -1;
    snprintf(s_qso_tx_message, sizeof s_qso_tx_message, "CQ %s %s", station_call(), station_grid());
    s_qso_tx_pending = true;
    s_qso_tx_seen = false;
    snprintf(s_auto_status, sizeof s_auto_status, "CQ TX QUEUED - %lu HZ EVEN", (unsigned long)s_cq_tx_audio_hz);
    return true;
}

static void qso_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10,14,30));
    text(12, 8, 2, COL_ACCENT, s_mode == DSP_MODE_FT4 ? "FT4 QSO" : "FT8 QSO");
    /* Compact TX monitor volume control in header, left of LOG. */
    char mon_label[24];
    text(237, 9, 1, COL_TEXT, "TX MON");
    rect(TX_MON_X, TX_MON_Y, TX_MON_W, 7, COL_TRACK);
    rect(TX_MON_X, TX_MON_Y, TX_MON_W * s_tx_mon_vol / 100, 7, COL_ACCENT);
    int mon_knob = TX_MON_X + TX_MON_W * s_tx_mon_vol / 100;
    rect(mon_knob - 2, TX_MON_Y - 3, 5, 13, COL_LINE);
    snprintf(mon_label, sizeof mon_label, "%d%%", s_tx_mon_vol);
    text(508, 9, 1, COL_LINE, mon_label);
    rect(530, 5, 98, 30, COL_BTN);
    text(553, 13, 2, COL_TEXT, "LOG");
    /* CQ TX audio offset, directly below LOG. +/- adjust by 50 Hz. */
    char cq_tx_label[24];
    snprintf(cq_tx_label, sizeof cq_tx_label, "TX %lu Hz", (unsigned long)s_cq_tx_audio_hz);
    rect(530, 39, 98, 15, COL_PANEL);
    text(535, 43, 1, COL_LINE, cq_tx_label);
    rect(530, 57, 46, 23, COL_BTN);
    rect(582, 57, 46, 23, COL_BTN);
    text(548, 63, 1, COL_TEXT, "-");
    text(600, 63, 1, COL_TEXT, "+");
    char b[120];
    snprintf(b, sizeof b, "%s  SNR:%+d  RX:%dHz  %s",
             s_qso_has_selected ? s_qso_call : station_call(),
             s_qso_has_selected ? (int)s_qso_selected.snr : 0,
             s_qso_has_selected ? (int)s_qso_selected.freq : (int)s_cq_tx_audio_hz,
             s_cq_active ? "CQ CALLING" : (s_auto ? "AUTO" : "MONITOR"));
    text(12, 37, 1, COL_LINE, b);
    char cq_preview[48];
    snprintf(cq_preview, sizeof cq_preview, "CQ %s %s", station_call(), station_grid());
    snprintf(b, sizeof b, "NEXT: %s", s_cq_active || !s_qso_has_selected ? cq_preview : s_qso_preview);
    text(12, 56, 2, COL_ACCENT, b);
    text(12, 83, 1, COL_TEXT, s_cq_active ? s_cq_debug : "QSO HISTORY (RX GREEN / TX YELLOW):");
    rect(10, 98, 620, 111, RGB565(17,25,43));
    int first = s_qso_history_n > 7 ? s_qso_history_n - 7 : 0;
    for (int i = first; i < s_qso_history_n; ++i) {
        const qso_history_t *h = &s_qso_history[i];
        int y = 102 + (i - first) * 15;
        snprintf(b, sizeof b, "%s %s  %-39.39s", h->stamp,
                 h->tx ? "TX" : "RX", h->message);
        text(15, y, 1, h->tx ? RGB565(255,211,91) : RGB565(111,234,171), b);
        if (!h->tx) {
            char sn[16];
            snprintf(sn, sizeof sn, "%+d dB", h->snr);
            text(572, y, 1, COL_LINE, sn);
        }
    }
    /* Show the step actually on air from the first RF symbol, rather than
     * leaving the previous/next sequence step highlighted until TX ends. */
    const bool qso_rf_tx = si5351_relay_is_tx() && si5351_tone_test_index() >= 0;
    const int highlighted_step = (qso_rf_tx && s_qso_tx_step >= 0)
                                 ? s_qso_tx_step : s_qso_step;
    for (int i = 0; i < 5; ++i) {
        int x = 12 + i * 125;
        rect(x, 218, 117, 30, i == highlighted_step ? COL_ACCENT : COL_BTN);
        text(x + 9, 228, 1, i == highlighted_step ? RGB565(0,0,0) : COL_TEXT, s_qso_steps[i]);
    }
    rect(12, 256, 160, 27, s_auto ? COL_ACCENT : COL_BTN);
    text(19, 265, 1, s_auto ? COL_BG : COL_TEXT,
         s_auto ? "AUTO SEQ: ON" : "AUTO SEQ: ARM");
    text(183, 260, 1, s_qso_error ? RGB565(255,100,100) : COL_LINE, s_auto_status);
    snprintf(b, sizeof b, "TX RF: %llu Hz  %s",
             (unsigned long long)(si5351_ft8_get_base_cHz()/100ULL),
             si5351_tone_test_running() ? "TX QUEUED/ACTIVE" : "READY");
    text(183, 275, 1, COL_LINE, b);
    rect(480, 256, 148, 27, s_auto_cq ? COL_ACCENT : COL_BTN);
    text(490, 265, 1, s_auto_cq ? COL_BG : COL_TEXT,
         s_auto_cq ? "AUTO QSO: ON" : "AUTO QSO: OFF");
    rect(12, 299, 142, 51, COL_BTN);
    text(49, 317, 2, COL_TEXT, "SEND");
    rect(168, 299, 142, 51, COL_BTN);
    text(205, 317, 2, COL_TEXT, "STOP");
    rect(324, 299, 142, 51, s_cq_active ? COL_ACCENT : COL_BTN_HI);
    text(374, 317, 2, s_cq_active ? COL_BG : COL_TEXT, "CQ");
    rect(480, 299, 148, 51, COL_BTN);
    text(518, 317, 2, COL_TEXT, "BACK");
}
static bool qso_touch(int x, int y)
{
    /* Shared FT8/FT4 QSO controls: handle BEFORE the FT4-specific branch.
     * Otherwise FT4 consumes touches and displays the misleading pending status. */
    if (x >= 530 && x < 628 && y >= 57 && y < 80) {
        if (si5351_tone_test_running() || si5351_relay_is_tx()) {
            snprintf(s_auto_status, sizeof s_auto_status,
                     "WAIT UNTIL TX FINISHES TO CHANGE CQ FREQ");
            return true;
        }
        uint32_t next = s_cq_tx_audio_hz;
        if (x < 576) {
            if (next >= CQ_TX_AUDIO_MIN + CQ_TX_AUDIO_STEP) next -= CQ_TX_AUDIO_STEP;
        } else if (x >= 582) {
            if (next <= CQ_TX_AUDIO_MAX - CQ_TX_AUDIO_STEP) next += CQ_TX_AUDIO_STEP;
        }
        if (next != s_cq_tx_audio_hz) {
            s_cq_tx_audio_hz = next;
            cq_tx_audio_save();
            if (!s_qso_has_selected || s_qso_own_cq) {
                esp_err_t err = si5351_ft8_set_base_cHz(
                    ((uint64_t)s_center + s_cq_tx_audio_hz) * 100ULL);
                if (err == ESP_OK)
                    s_qso_rf_base_chz = ((uint64_t)s_center + s_cq_tx_audio_hz) * 100ULL;
            }
            snprintf(s_auto_status, sizeof s_auto_status,
                     "CQ TX AUDIO: %lu HZ (SAVED)", (unsigned long)s_cq_tx_audio_hz);
        }
        return true;
    }
    if (y >= 10 && y < 35 && x >= TX_MON_X - 6 && x <= TX_MON_X + TX_MON_W + 6) {
        s_tx_mon_drag = true;
        (void)tx_mon_volume_touch(x);
        return true;
    }
    if (x >= 530 && x < 628 && y >= 5 && y < 35) {
        s_ft8_log_page = 0; s_ft8_log_selected = -1;
        s_ft8_log_confirm = false; s_ft8_log_notice[0] = 0;
        ft8_log_refresh(); s_menu = 13; return true;
    }
    if (s_mode == DSP_MODE_FT4) {
        /* Phase 4: FT4 one-shot manual SEND/CQ only. Never arm FT8 AUTO SEQ. */
        if (y >= 218 && y < 248 && x >= 12 && x < 637 &&
            s_qso_has_selected && !si5351_tone_test_running()) {
            int idx = (x - 12) / 125;
            if (idx >= 0 && idx < 5 && (x - 12) % 125 < 117) {
                s_qso_step = idx;
                qso_prepare();
            }
            return true;
        }
        if (y >= 256 && y < 283 && x >= 12 && x < 172 && s_qso_has_selected) {
            if (s_qso_wait_cq || s_auto_tx_parity < 0) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 AUTO - SELECT FRESH CQ");
            } else if (!si5351_tone_test_running()) {
                s_auto_enabled = true;
                s_auto = true;
                s_auto_done = false;
                s_auto_final_queued = false;
                s_auto_final_started = false;
                s_auto_attempts = 0;
                s_auto_last_seq = s_qso_selected.seq;
                s_auto_last_queued_slot = -1;
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 AUTO SEQ ARMED");
            }
            return true;
        }
        if (y >= 256 && y < 283 && x >= 480 && x < 628) {
            s_auto_cq = !s_auto_cq;
            snprintf(s_auto_status, sizeof s_auto_status,
                     "FT4 AUTO QSO %s", s_auto_cq ? "ENABLED" : "DISABLED");
            return true;
        }
        if (y >= 299 && y < 350 && x >= 480 && x < 628) {
            s_menu = 0;
            s_panel_mode = -1;
            return true;
        }
        if (y >= 299 && y < 350 && x >= 168 && x < 310) {
            si5351_tone_test_stop();
            s_cq_active = false;
            s_auto = false;
            s_auto_enabled = false; /* explicit STOP disables auto until re-armed */
            s_auto_cq = false;
            s_qso_has_selected = false;
            s_qso_call[0] = 0;
            ft4_set_qso_focus(NULL, 0);
            snprintf(s_auto_status, sizeof s_auto_status, "FT4 STOPPED");
            return true;
        }
        if (y >= 299 && y < 350 && x >= 12 && x < 154) {
            if (!s_qso_has_selected || s_qso_wait_cq || !s_qso_selected.utc_slot) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4: SELECT FRESH CQ FIRST");
                return true;
            }
            if (si5351_tone_test_running() || si5351_relay_is_tx()) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 TX BUSY");
                return true;
            }
            struct timeval now;
            gettimeofday(&now, NULL);
            int64_t utc_us = (int64_t)now.tv_sec * 1000000LL + now.tv_usec;
            int64_t current_slot = utc_us / 7500000LL;
            int64_t age = current_slot - (int64_t)s_qso_selected.utc_slot;
            if (now.tv_sec < 1760000000LL || age < 0 || age > 4) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 CQ TOO OLD - SELECT NEW CQ");
                return true;
            }
            char to[16], from[16], extra[16];
            if (sscanf(s_qso_preview, "%15s %15s %15s", to, from, extra) != 3) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 INVALID MESSAGE");
                return true;
            }
            esp_err_t err = qso_sync_tx_frequency();
            if (err == ESP_OK) err = si5351_ft4_set_message(to, from, extra);
            if (err == ESP_OK) err = si5351_ft4_start_parity((int)((s_qso_selected.utc_slot + 1u) & 1u));
            s_qso_error = (err != ESP_OK);
            if (err == ESP_OK) {
                qso_history_queue_tx();
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 SEND QUEUED - ONE FRAME");
            } else {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 SEND: %s", esp_err_to_name(err));
            }
            return true;
        }
        if (y >= 299 && y < 350 && x >= 324 && x < 466) {
            if (si5351_tone_test_running() || si5351_relay_is_tx()) {
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 TX BUSY - PRESS STOP");
                return true;
            }
            esp_err_t err = si5351_ft8_set_base_cHz(((uint64_t)s_center + s_cq_tx_audio_hz) * 100ULL);
            if (err == ESP_OK) err = si5351_ft4_set_message("CQ", station_call(), station_grid());
            if (err == ESP_OK) err = si5351_ft4_start_parity(s_auto_cq ? 0 : -1);
            s_qso_error = (err != ESP_OK);
            if (err == ESP_OK) {
                /* Manual CQ becomes repeated AUTO CQ only when AUTO QSO is ON. */
                if (s_auto_cq) {
                    s_cq_active = true;
                    s_qso_has_selected = false;
                    s_qso_call[0] = 0;
                    s_qso_own_cq = false;
                    s_qso_step = 1;
                    s_qso_history_n = 0;
                    s_qso_history_last_seq = 0;
                    s_auto = false;
                    s_cq_tx_parity = 0;
                    /* First manually queued CQ uses next available slot.
                     * Do not queue another CQ until it has completed. */
                    s_cq_last_queued_slot = -1;
                    ft8_msg_t cq_msgs[24];
                    ft8_status_t cq_st;
                    int cq_n = ft4_get_messages(cq_msgs, 24, &cq_st);
                    s_cq_last_rx_seq = cq_n ? cq_msgs[cq_n - 1].seq : 0;
                    ft4_set_qso_focus(NULL, 0);
                }
                snprintf(s_auto_status, sizeof s_auto_status,
                         s_auto_cq ? "FT4 AUTO CQ ARMED" : "FT4 CQ QUEUED - ONE FRAME");
            }
            else
                snprintf(s_auto_status, sizeof s_auto_status, "FT4 CQ ERROR: %s", esp_err_to_name(err));
            return true;
        }
        /* Unassigned FT4 QSO touch: preserve the current status message. */
        return false;
    }
    if (y >= 218 && y < 248 && x >= 12 && x < 637 && !s_auto && !s_cq_active && s_qso_has_selected) {
        int idx = (x - 12) / 125;
        if (idx >= 0 && idx < 5 && (x - 12) % 125 < 117 &&
            !si5351_tone_test_running()) {
            s_qso_step = idx;
            s_qso_error = false;
            qso_prepare();
        }
        return true;
    }
    if (y >= 256 && y < 283 && x >= 480 && x < 628) {
        s_auto_cq = !s_auto_cq;
        snprintf(s_auto_status, sizeof s_auto_status, "%s",
                 s_auto_cq ? "AUTO QSO ENABLED" : "AUTO QSO DISABLED");
        return true;
    }
    if (y >= 256 && y < 283 && x >= 12 && x < 172 && s_qso_has_selected && !s_cq_active) {
        /* AUTO SEQ is an ARM button, not a toggle: pressing it when
         * already ON must never cancel the first CQ response.
         * Use STOP to disable AUTO explicitly. */
        if (s_auto && s_auto_enabled) {
            snprintf(s_auto_status, sizeof s_auto_status,
                     "AUTO ALREADY ON - WAIT TX SLOT");
        }
        else if (s_qso_wait_cq) {
            snprintf(s_auto_status, sizeof s_auto_status,
                     "MONITOR - WAIT FOR FRESH CQ BEFORE ARM");
        }
        else if (!si5351_tone_test_running()) {
            s_auto_enabled = true;
            s_auto = true;
            s_auto_done = false;
            s_auto_final_queued = false;
            s_auto_final_started = false;
            s_auto_attempts = 0;
            /* Keep parity determined from the selected CQ slot. */
            s_auto_last_queued_slot = -1;
            s_auto_last_seq = s_qso_selected.seq;
            snprintf(s_auto_status, sizeof s_auto_status, "AUTO ARMED - WAIT TX SLOT");
        }
        return true;
    }
    if (y >= 299 && y < 350) {
        if (x >= 12 && x < 154 && s_qso_has_selected && !s_cq_active) {
            /* Manual SEND joins the current FT8 slot, skipping elapsed symbols. */
            if (si5351_relay_is_tx() || si5351_tone_test_index() >= 0) {
                snprintf(s_auto_status, sizeof s_auto_status, "SEND: TX ALREADY ACTIVE");
                return true;
            }
            if (s_qso_wait_cq) {
                snprintf(s_auto_status, sizeof s_auto_status,
                         "MONITOR - WAIT FOR CQ BEFORE TX");
                return true;
            }
            if (s_auto_tx_parity < 0) {
                s_qso_error = true;
                snprintf(s_auto_status, sizeof s_auto_status, "TX SLOT UNKNOWN - SELECT FRESH CQ");
                return true;
            }
            char to[16], from[16], extra[16];
            if (sscanf(s_qso_preview, "%15s %15s %15s", to, from, extra) == 3) {
                bool queued_auto = si5351_tone_test_running();
                esp_err_t err = ESP_OK;
                if (queued_auto) {
                    /* AUTO has queued a future frame: replace it with a late
                     * manual SEND, keeping the original UTC symbol timeline. */
                    err = si5351_ft8_takeover_queued_send(to, from, extra);
                } else {
                    err = qso_sync_tx_frequency();
                    if (err == ESP_OK) err = si5351_ft8_set_message(to, from, extra);
                    if (err == ESP_OK) err = si5351_tone_test_start_immediate();
                }
                if (err == ESP_OK) {
                    qso_history_queue_tx();
                    /* A manual late SEND occupies the CURRENT UTC TX slot,
                     * not the next matching slot. Marking a future slot here
                     * would prevent AUTO SEQ from resuming after this frame. */
                    struct timeval send_tv;
                    gettimeofday(&send_tv, NULL);
                    int64_t send_slot = (int64_t)send_tv.tv_sec / 15;
                    s_auto_last_queued_slot = send_slot;
                    /* Manual SEND is a one-frame override, not AUTO OFF.
                     * After the TX worker finishes, qso_auto_tick() resumes
                     * monitoring replies and scheduling the next QSO step. */
                    s_auto_enabled = true;
                    s_auto = true;
                    s_auto_done = false;
                    if (!s_qso_start_utc) s_qso_start_utc = (time_t)(send_slot * 15);
                    if (s_qso_step == 4 && s_auto) {
                        s_auto_final_queued = true;
                        s_auto_final_started = false;
                    }
                    ++s_auto_attempts;
                    snprintf(s_auto_status, sizeof s_auto_status,
                             "SEND TX - AUTO SEQ CONTINUES");
                } else {
                    snprintf(s_auto_status, sizeof s_auto_status,
                             "SEND ERROR: %s", esp_err_to_name(err));
                }
                s_qso_error = (err != ESP_OK);
            } else {
                s_qso_error = true;
                snprintf(s_auto_status, sizeof s_auto_status, "SEND ERROR: INVALID MESSAGE");
            }
        } else if (x >= 168 && x < 310) {
            s_cq_active = false;
            s_qso_own_cq = false;
            s_auto_cq = false;
            /* STOP cancels the current QSO only. Keep AUTO SEQ armed for the next CQ selection. */
            s_auto_enabled = true;
            qso_auto_off("STOPPED - AUTO OFF");
            si5351_tone_test_stop();
        } else if (x >= 324 && x < 466) {
            if (s_cq_active) {
                s_cq_active = false;
                si5351_tone_test_stop();
                snprintf(s_auto_status, sizeof s_auto_status, "CQ STOPPED");
            } else if (!s_auto && !si5351_tone_test_running()) {
                s_cq_active = true;
                s_qso_own_cq = false;
                /* CQ initiator: first reply is REPORT, never CALL. */
                s_qso_step = 1;
                s_cq_seen_count = 0;
                snprintf(s_cq_debug, sizeof s_cq_debug, "CQ ARMED - WAIT RX");
                s_qso_has_selected = false;
                s_qso_call[0] = 0;
                s_qso_history_n = 0;
                s_qso_history_last_seq = 0;
                s_qso_tx_pending = false;
                s_qso_tx_step = -1;
                s_cq_last_queued_slot = -1;
                ft8_msg_t cq_msgs[24];
                ft8_status_t cq_st;
                int cq_n = ft8_get_messages(cq_msgs, 24, &cq_st);
                s_cq_last_rx_seq = cq_n ? cq_msgs[cq_n - 1].seq : 0;
                ft8_set_qso_focus(NULL, 0);
                s_qso_error = false;
                snprintf(s_auto_status, sizeof s_auto_status, "CQ ARMED - %lu HZ EVEN", (unsigned long)s_cq_tx_audio_hz);
            }
        } else if (x >= 480 && x < 628) {
            /* BACK only closes the QSO screen; AUTO continues in background. */
            s_menu = 0;
            s_panel_mode = -1;
        }
    }
    return true;
}

/* Large FT8 return button: same 46x44 touch size as VOL +. */
#define FT8_BACK_X 590
#define FT8_BACK_Y 112
#define FT8_BACK_W 46
#define FT8_BACK_H 44

#define FT8_FILTER_X FT8_BACK_X
#define FT8_FILTER_Y (FT8_BACK_Y + FT8_BACK_H + 4)
#define FT8_FILTER_W FT8_BACK_W
#define FT8_FILTER_H 44

static void ft8_draw_filter_button(void)
{
    rect(FT8_FILTER_X, FT8_FILTER_Y, FT8_FILTER_W, FT8_FILTER_H, COL_BTN_HI);
    text(FT8_FILTER_X + (s_ft8_cq_only ? 11 : 5), FT8_FILTER_Y + 15, 2,
         COL_TEXT, s_ft8_cq_only ? "CQ" : "ALL");
}

static void ft8_draw_back_button(void)
{
    rect(FT8_BACK_X, FT8_BACK_Y, FT8_BACK_W, FT8_BACK_H,
         s_auto ? COL_ACCENT : COL_BTN_HI);
    text(FT8_BACK_X + 5, FT8_BACK_Y + 15, 2,
         s_auto ? COL_BG : COL_TEXT, "QSO");
}

/* FT4 RX panel with shared QSO tracking and automatic sequencing. */
static void draw_ft4_panel(void)
{
    ft8_msg_t msgs[PANEL_COLS * PANEL_ROWS];
    ft8_status_t st = {0};
    int n = ft8_visible_messages(msgs, PANEL_COLS * PANEL_ROWS, &st);
    bool body = panel_begin(DSP_MODE_FT4, s_ft8_visible_period_revision);
    char b[100];
    if (st.locked)
        snprintf(b, sizeof b, "FT4 RX  SLOT SYNC OK  T %.1f S  MSGS %u", (double)st.slot_pos, (unsigned)st.total);
    else
        snprintf(b, sizeof b, "FT4 RX  SCANNING  MSGS %u", (unsigned)st.total);
    text(6, WF_Y0 + 3, 1, st.locked ? COL_ACCENT : COL_LINE, b);
    if (!body) { ft8_draw_filter_button(); ft8_draw_back_button(); return; }
    if (!n) {
        text(6, WF_Y0 + PANEL_HDR_H + 4, 1, COL_TEXT, "FT4: TAP A DECODED STATION TO MONITOR QSO");
        ft8_draw_filter_button();
        ft8_draw_back_button();
        return;
    }
    /* FT4 uses the same ADIF worked-before lookup and CQ colours as FT8. */
    ft8_b4_ensure_loaded();
    uint8_t band = ft8_b4_band((double)s_center / 1000000.0);
    panel_separators();
    for (int k = 0; k < n; ++k) {
        const ft8_msg_t *msg = &msgs[n - 1 - k];
        snprintf(b, sizeof b, "%4d %+3d %.*s", (int)msg->freq, (int)msg->snr, PANEL_TEXT_CH, msg->text);
        uint16_t color = COL_TEXT;
        if (ft8_is_cq_message(msg->text)) {
            char cq_call[FT8_B4_CALL_LEN] = "";
            char a[20] = "", bcall[20] = "", ccall[20] = "";
            int parts = sscanf(msg->text, "%19s %19s %19s", a, bcall, ccall);
            const char *candidate = (parts >= 3 && (!strcmp(bcall, "DX") ||
                !strcmp(bcall, "TEST"))) ? ccall : (parts >= 2 ? bcall : "");
            if (*candidate && strlen(candidate) < sizeof cq_call &&
                strpbrk(candidate, "0123456789") &&
                strpbrk(candidate, "ABCDEFGHIJKLMNOPQRSTUVWXYZ"))
                snprintf(cq_call, sizeof cq_call, "%s", candidate);
            if (cq_call[0] && s_ft8_b4_loaded && band)
                color = ft8_b4_find(cq_call, band) ?
                    RGB565(255, 145, 145) : RGB565(70, 255, 110);
            else
                color = RGB565(255, 220, 0);
        }
        panel_line(k, n, color, b);
    }
    ft8_draw_filter_button();
    ft8_draw_back_button();
}

static void draw_ft8_panel(void)
{
    /* Force CQ colour refresh whenever the tuned band changes, even if the
     * decoder message count remains identical after retuning. */
    static uint8_t last_b4_band = 255;
    uint8_t current_b4_band = ft8_b4_band((double)s_center / 1000000.0);
    if (last_b4_band != current_b4_band) {
        last_b4_band = current_b4_band;
        s_panel_mode = -1;
    }
    ft8_b4_ensure_loaded();
    ft8_msg_t msgs[PANEL_COLS * PANEL_ROWS];
    ft8_status_t st;
    int n = ft8_visible_messages(msgs, PANEL_COLS * PANEL_ROWS, &st); /* oldest first */
    bool body = panel_begin(DSP_MODE_FT8, s_ft8_visible_period_revision);

    char b[100];
    if (st.locked)
        snprintf(b, sizeof b, "FT8  SLOT SYNC OK   T %04.1f S   SLOTS %u   MSGS %u", (double)st.slot_pos,
                 (unsigned)st.slots, (unsigned)st.total);
    else
        snprintf(b, sizeof b, "FT8  SCANNING FOR THE SLOT TIMING   MSGS %u", (unsigned)st.total);
    text(6, WF_Y0 + 3, 1, st.locked ? RGB565(60, 220, 140) : RGB565(255, 190, 60), b);
    if (!body) { ft8_draw_back_button(); ft8_draw_filter_button(); return; }

    if (n == 0) {
        text(6, WF_Y0 + 3 + PANEL_HDR_H, 1, RGB565(170, 170, 170), s_ft8_cq_only ? "NO CQ CALLS IN LAST DECODED PERIOD" : "NO MESSAGES YET. TUNE TO AN FT8 FREQUENCY IN USB,");
        text(6, WF_Y0 + 3 + PANEL_HDR_H + 10, 1, RGB565(170, 170, 170), s_ft8_cq_only ? "PRESS CQ TO RETURN TO ALL MESSAGES" : "E.G. 14.074 MHZ (20 M). THE FIRST DECODE SETS THE SLOT TIMING.");
        ft8_draw_back_button(); ft8_draw_filter_button();
        return;
    }
    panel_separators();
    uint16_t newest_slot = msgs[n - 1].slot;
    for (int k = 0; k < n; k++) {
        const ft8_msg_t *m = &msgs[n - 1 - k]; /* k = 0 is the newest */
        snprintf(b, sizeof b, "%4d %+3d %.*s", (int)m->freq, (int)m->snr, PANEL_TEXT_CH, m->text);
        uint16_t color = (m->slot == newest_slot) ? RGB565(235, 235, 235) : RGB565(120, 120, 120);
        if (ft8_is_cq_message(m->text)) {
            char cq_call[FT8_B4_CALL_LEN] = "";
            char a[20] = "", bcall[20] = "", ccall[20] = "";
            int parts = sscanf(m->text, "%19s %19s %19s", a, bcall, ccall);
            const char *candidate = (parts >= 3 && (!strcmp(bcall, "DX") ||
                !strcmp(bcall, "TEST"))) ? ccall : (parts >= 2 ? bcall : "");
            if (*candidate && strlen(candidate) < sizeof cq_call &&
                strpbrk(candidate, "0123456789") && strpbrk(candidate, "ABCDEFGHIJKLMNOPQRSTUVWXYZ"))
                snprintf(cq_call, sizeof cq_call, "%s", candidate);
            if (cq_call[0] && s_ft8_b4_loaded && current_b4_band)
                color = ft8_b4_find(cq_call, current_b4_band) ?
                    RGB565(255, 145, 145) : RGB565(70, 255, 110);
            else if (m->slot == newest_slot)
                color = RGB565(255, 220, 0); /* Unknown: retain existing yellow */
        }
        panel_line(k, n, color, b);
    }
    ft8_draw_back_button(); ft8_draw_filter_button();
}

static void draw_wspr_panel(void)
{
    wspr_msg_t msgs[PANEL_COLS * PANEL_ROWS];
    wspr_status_t st;
    int n = wspr_get_messages(msgs, PANEL_COLS * PANEL_ROWS, &st); /* oldest first */
    bool body = panel_begin(DSP_MODE_WSPR, st.total + st.other_types + 1000u * (uint32_t)st.selftest_state);

    char b[160];
    if (st.selftest_state == 1) {
        snprintf(b, sizeof b, "WSPR SELF TEST RUNNING %d %%  (SYNTHETIC SIGNAL THROUGH THE REAL DECODER)", st.selftest_pct);
    } else {
        if (st.grid_known)
            snprintf(b, sizeof b, "WSPR  T %03.0f S  SPOTS %u  BUF %.0f S", (double)st.cycle_pos, (unsigned)st.total,
                     (double)st.buffered_s);
        else
            snprintf(b, sizeof b, "WSPR  LISTENING  BUF %.0f S OF 115  SPOTS %u", (double)st.buffered_s, (unsigned)st.total);
        size_t l = strlen(b);
        if (st.other_types) { snprintf(b + l, sizeof b - l, "  OTHER %u", (unsigned)st.other_types); l = strlen(b); }
        /* where does the chain stop?  SC = sync scans, CAND = candidates tried, FANO = decodes
         * that converged, BEST = strongest sync correlation of the last scan (0.15 = threshold),
         * DROP = audio samples lost */
        snprintf(b + l, sizeof b - l, "   SC %u CAND %u FANO %u BEST %.2f DROP %u", (unsigned)st.scans, (unsigned)st.cands,
                 (unsigned)st.fano_ok, (double)st.best_m, (unsigned)st.drops);
        l = strlen(b);
        if (st.selftest_state == 2) snprintf(b + l, sizeof b - l, "   TEST OK %.0fX", (double)st.selftest_speed);
        else if (st.selftest_state == 3) snprintf(b + l, sizeof b - l, "   TEST FAILED");
    }
    uint16_t hc = st.selftest_state == 3 ? RGB565(255, 90, 90) : (st.grid_known ? RGB565(240, 170, 80) : RGB565(255, 190, 60));
    text(6, WF_Y0 + 3, 1, hc, b);
    if (!body) return;

    if (n == 0) {
        int y = WF_Y0 + 3 + PANEL_HDR_H;
        text(6, y, 1, RGB565(170, 170, 170), "NO SPOTS YET. TUNE TO A WSPR DIAL FREQUENCY IN USB, E.G. 14.0956 MHZ (20 M).");
        text(6, y + 10, 1, RGB565(170, 170, 170), "A SPOT APPEARS WHEN A TRANSMISSION ENDS; THE FIRST ONE NEEDS ABOUT 4 MIN. DO NOT RETUNE");
        text(6, y + 20, 1, RGB565(170, 170, 170), "WHILE WAITING: THE SIGNAL MUST STAY AT THE SAME AUDIO FREQUENCY FOR 2 MINUTES.");
        text(6, y + 40, 1, RGB565(120, 200, 255), "TAP THIS PANEL TO RUN A SELF TEST (SYNTHETIC SIGNAL, ABOUT 1 MIN, CHECKS THE DECODER ITSELF).");
        return;
    }
    panel_separators();
    for (int k = 0; k < n; k++) {
        const wspr_msg_t *m = &msgs[n - 1 - k];
        snprintf(b, sizeof b, "%4d %+3d %.*s", (int)lroundf(m->freq), (int)m->snr, PANEL_TEXT_CH, m->text);
        panel_line(k, n, k < PANEL_ROWS ? RGB565(235, 235, 235) : RGB565(120, 120, 120), b);
    }
}

static void draw_aprs_panel(void)
{
    rect(0, SPEC_Y0, LW, WF_Y0 + WF_H - SPEC_Y0, COL_BG);
    char b[150];
    snprintf(b, sizeof b, "APRS AFSK1200  %.3f MHZ", s_center / 1e6);
    rs41_smooth_text(8, SPEC_Y0 + 3, COL_ACCENT, b);
    snprintf(b, sizeof b, "OK %lu  CRC ERR %lu  FLAGS %lu  AFSK %d%%", (unsigned long)aprs_packet_count(),
             (unsigned long)aprs_crc_errors(), (unsigned long)aprs_flags(), aprs_quality_pct());
    rs41_smooth_text(8, SPEC_Y0 + 23, COL_LINE, b);
    aprs_packet_t pk[APRS_HISTORY];
    int n = aprs_packets(pk, APRS_HISTORY);
    if (!n) { rs41_smooth_text(8, SPEC_Y0 + 51, COL_TEXT, "WAITING FOR APRS PACKET..."); return; }
    for (int i = 0; i < n; ++i) {
        char line[101];
        snprintf(line, sizeof line, "%.62s", pk[i].text);
        line[100] = 0;
        rs41_smooth_text(8, SPEC_Y0 + 44 + i * 19, i ? COL_TEXT : COL_ACCENT, line);
    }
}

static void draw_lora_panel(void)
{
    rect(0, SPEC_Y0, LW, WF_Y0 + WF_H - SPEC_Y0, COL_BG);
    text(8, SPEC_Y0 + 3, 2, COL_ACCENT, "LORA APRS RX  433.775 MHZ  SF12 BW125");
    char b[192];
    snprintf(b, sizeof b, "RECEIVED: %lu  (LAST 5)", (unsigned long)lora_aprs_packet_count());
    text(8, SPEC_Y0 + 24, 1, COL_LINE, b);
    lora_aprs_packet_t p[LORA_APRS_HISTORY];
    int n = lora_aprs_packets(p, LORA_APRS_HISTORY);
    if (!n) {
        lora_aprs_status(b, sizeof b);
        b[61] = 0;
        rs41_smooth_text(8, SPEC_Y0 + 47, COL_TEXT, b);
        rs41_smooth_text(8, SPEC_Y0 + 68, COL_ACCENT, "WAITING FOR APRS PACKET...");
        return;
    }
    for (int i = 0; i < n; ++i) {
        int y = SPEC_Y0 + 43 + i * 25;
        snprintf(b, sizeof b, "%d RSSI:%d SNR:%+.1f", i + 1, p[i].rssi_dbm, (double)p[i].snr_db);
        text(8, y, 1, i == 0 ? COL_LINE : COL_ACCENT, b);
        snprintf(b, sizeof b, "%s", p[i].text);
        b[61] = 0;
        rs41_smooth_text(8, y + 9, COL_TEXT, b);
    }
}

static uint32_t view_span_hz(void)
{
    uint32_t full = dsp_span_hz();
    if (!s_view_span || s_view_span > full) s_view_span = full;
    return s_view_span;
}

static void draw_spectrum(const float *spec, int bins)
{
    if (lora_aprs_enabled()) { draw_lora_panel(); return; }
    if (s_mode == DSP_MODE_APRS) { draw_aprs_panel(); return; }
    const bool rs = is_panel_mode(s_mode); /* mode with a text panel instead of the waterfall */
    if (!rs)
        memmove(LBUF + (size_t)(WF_Y0 + 1) * LW, LBUF + (size_t)WF_Y0 * LW,
                (size_t)(WF_H - 1) * LW * sizeof(uint16_t));
    int cx = LW / 2;
    uint32_t full = dsp_span_hz(), visible = view_span_hz();
    int shown_bins = (int)((uint64_t)bins * visible / full);
    if (shown_bins < 2) shown_bins = 2;
    int start_bin = (bins - shown_bins) / 2;
    for (int x = 0; x < LW; x++) {
        int bin = start_bin + (int)((int64_t)x * shown_bins / LW);
        if (bin >= bins) bin = bins - 1;
        float tt = (spec[bin] - DB_MIN) / (DB_MAX - DB_MIN);
        if (!rs) LBUF[(size_t)WF_Y0 * LW + x] = heat(tt);
        int h = (int)(SPEC_H * (tt < 0 ? 0 : tt > 1 ? 1 : tt));
        int ytop = SPEC_Y0 + SPEC_H - h;
        for (int y = SPEC_Y0; y < SPEC_Y0 + SPEC_H; y++)
            LBUF[(size_t)y * LW + x] = (y >= ytop) ? COL_ACCENT : COL_BG;
    }
    for (int y = SPEC_Y0; y < (rs ? SPEC_Y0 + SPEC_H : WF_Y0 + WF_H); y++) LBUF[(size_t)y * LW + cx] = COL_LINE;
    if (s_mode == DSP_MODE_RS41) draw_rs41_panel();
    else if (s_mode == DSP_MODE_FT8) draw_ft8_panel();
    else if (s_mode == DSP_MODE_FT4) draw_ft4_panel();
    else if (s_mode == DSP_MODE_WSPR) draw_wspr_panel();
    if (lora_aprs_enabled() || s_mode == DSP_MODE_HORUS) {
        char msg[192];
        if (s_mode == DSP_MODE_HORUS) {
            /* HORUS: six medium-font lines (1.5x) with more vertical spacing.
             * Keep FT8/WSPR, other UI panels and decoder logic untouched. */
            const int hx = 8, hy = WF_Y0 + 2, hp = 15;
            rect(4, WF_Y0 + 2, 632, WF_H - 4, COL_BG);
            horus_decoder_status(msg, sizeof msg);
            msg[62] = 0; /* 62 glyphs * 10 px = 620 px */
            rs41_smooth_text(hx, hy, COL_ACCENT, msg);
            horus_v1_telemetry_t v;
            if (horus_decoder_get_telemetry(&v)) {
                const char *payload_name = (v.version == 3) ? v.callsign : horus_payload_name(v.payload_id);
                if (v.version == 3) {
                    snprintf(msg, sizeof msg, "V3  %s  PKT:%u  CRC:OK",
                             payload_name ? payload_name : "-", v.sequence);
                } else if (payload_name) {
                    snprintf(msg, sizeof msg, "V%u  %s  ID:%u  PKT:%u  CRC:OK",
                             v.version, payload_name, v.payload_id, v.sequence);
                } else {
                    snprintf(msg, sizeof msg, "V%u  ID:%u  PKT:%u  CRC:OK",
                             v.version, v.payload_id, v.sequence);
                }
                msg[62] = 0;
                rs41_smooth_text(hx, hy + hp, COL_LINE, msg);
                snprintf(msg, sizeof msg, "UTC %02u:%02u:%02u", v.hour, v.minute, v.second);
                rs41_smooth_text(hx, hy + 2 * hp, COL_TEXT, msg);
                snprintf(msg, sizeof msg, "LAT %.6f  LON %.6f",
                         (double)v.latitude, (double)v.longitude);
                msg[62] = 0;
                rs41_smooth_text(hx, hy + 3 * hp, COL_TEXT, msg);
                snprintf(msg, sizeof msg, "ALT %u m  SPD %u km/h  SAT %u",
                         v.altitude, v.speed, v.satellites);
                msg[62] = 0;
                rs41_smooth_text(hx, hy + 4 * hp, COL_TEXT, msg);
                if (v.version == 3) {
                    char temp[24], bat[24];
                    if (v.v3_has_temperature) snprintf(temp, sizeof temp, "%d C", v.temperature);
                    else snprintf(temp, sizeof temp, "-");
                    if (v.v3_has_battery) snprintf(bat, sizeof bat, "%.2f V", v.v3_battery_mv / 1000.0);
                    else snprintf(bat, sizeof bat, "-");
                    snprintf(msg, sizeof msg, "TEMP %s  BAT %s", temp, bat);
                } else {
                    snprintf(msg, sizeof msg, "TEMP %d C  BAT %.2f V",
                             v.temperature, (double)(v.battery_raw * (5.0f / 255.0f)));
                }
                rs41_smooth_text(hx, hy + 5 * hp, COL_TEXT, msg);
            } else {
                char raw[260]; unsigned ver = 0, len = 0; bool parsed = false;
                horus_decoder_raw(raw, sizeof(raw), &ver, &len, &parsed);
                if (len) {
                    snprintf(msg, sizeof msg, "CRC OK  V%u  HEX:%u  PARSE:%s",
                             ver, len, parsed ? "OK" : "NE");
                    rs41_smooth_text(hx, hy + hp, COL_TEXT, msg);
                    raw[62] = 0;
                    rs41_smooth_text(hx, hy + 2 * hp, COL_TEXT, raw);
                } else {
                    rs41_smooth_text(hx, hy + 2 * hp, COL_TEXT, "WAITING FOR HORUS PACKET...");
                }
            }
        }
    }
}

/* ---- control band -------------------------------------------------------- */

typedef struct { int x, y, w, h; } box_t;
static const box_t B_TDN  = {  6, 220, 52, 44};  /* tune down one step */
static const box_t B_STEP = { 62, 220, 84, 44};  /* cycle step size    */
static const box_t B_TUP  = {150, 220, 52, 44};  /* tune up one step   */
static const box_t B_RS41 = {210, 220, 58, 44};  /* RS41 mode; pressed again in RS41 mode = centre on the strongest peak */
static const box_t B_GAIN = {272, 220, 44, 44};  /* tuner gain: AGC / manual steps */
static const box_t B_PLAN = {396, 220, 50, 44};  /* band-plan menu: time-scheduled FT8/WSPR band hopping */
static const box_t B_HF   = {  6, 272, 64, 44};
static const box_t B_VHF  = { 74, 272, 64, 44};
static const box_t B_UHF  = {142, 272, 64, 44};
static const box_t B_PSK  = {320, 220, 72, 44}; /* PSK Reporter upload checkbox */
static const box_t B_LORARX = {210, 272, 72, 44}; /* Direct LoRa RX entry */
static const box_t B_HORUS = {286, 272, 50, 44};
static const box_t B_LORA = {339, 272, 54, 44};
static const box_t B_APRS = {396, 272, 52, 44};
/* Mode row: WFM NFM AM USB LSB CW FT8 WSPR — 8 buttons, 52 px wide, 3 px gap (RS41 has its own
 * button). The row ends at x = 443; VOL/SQL start at x = 454. */
static const uint8_t MODE_ROW[] = {DSP_MODE_WFM, DSP_MODE_NFM, DSP_MODE_AM, DSP_MODE_USB, DSP_MODE_LSB, DSP_MODE_CW, DSP_MODE_FT8, DSP_MODE_FT4, DSP_MODE_WSPR};
#define N_MODE_ROW (int)(sizeof(MODE_ROW) / sizeof(MODE_ROW[0]))
#define MODE_BTN_W 46
#define MODE_BTN_X(i) (6 + (i) * (MODE_BTN_W + 3))
#define MODE_BTN_Y 322
#define MODE_BTN_H 36
static const box_t B_VDN  = {454, 226, 46, 44};
static const box_t B_VUP  = {590, 226, 46, 44};
static const box_t B_SDN  = {454, 300, 46, 44};
static const box_t B_SUP  = {590, 300, 46, 44};

static void draw_box(const box_t *b, const char *label, int sc, uint16_t fill)
{
    rect(b->x, b->y, b->w, b->h, fill);
    rect(b->x, b->y, b->w, 2, COL_BTN_HI);
    text(b->x + (b->w - text_w(label, sc)) / 2, b->y + b->h / 2 - 4 * sc, sc, COL_TEXT, label);
}

static void draw_controls(void)
{
    rect(0, CTRL_Y0, LW, LH - CTRL_Y0, COL_PANEL);
    char sl[12];
    step_label(sl, sizeof sl);
    draw_box(&B_TDN, "-", 3, COL_BTN);
    draw_box(&B_STEP, sl, 2, RGB565(70, 70, 110));
    draw_box(&B_TUP, "+", 3, COL_BTN);
    draw_box(&B_RS41, "RS41", 2, (s_mode == DSP_MODE_RS41) ? mode_color(DSP_MODE_RS41) : COL_BTN);
    {
        char gl[16];
        if (GAINS[s_gain_i] < 0) snprintf(gl, sizeof gl, "AGC");
        else snprintf(gl, sizeof gl, "G%d", (GAINS[s_gain_i] / 10) % 100); /* 2 digits */
        draw_box(&B_GAIN, gl, 2, GAINS[s_gain_i] < 0 ? COL_BTN : RGB565(90, 70, 40));
    }
    { /* band-plan button: green while a plan is armed, status line under the label */
        bool on = bandplan_active() != BP_OFF;
        rect(B_PLAN.x, B_PLAN.y, B_PLAN.w, B_PLAN.h, on ? RGB565(24, 110, 70) : COL_BTN);
        rect(B_PLAN.x, B_PLAN.y, B_PLAN.w, 2, COL_BTN_HI);
        text(B_PLAN.x + (B_PLAN.w - text_w("PLAN", 2)) / 2, B_PLAN.y + 4, 2, COL_TEXT, "PLAN");
        if (on) {
            bp_slot_t cur;
            char bn[8];
            if (bandplan_current(&cur)) {
                bandplan_band_name(bandplan_slot_freq(&cur), bn, sizeof bn);
                /* Narrow PLAN button: show short band identifier without overflow. */
                text(B_PLAN.x + (B_PLAN.w - text_w(bn, 1)) / 2, B_PLAN.y + 28, 1, RGB565(190, 255, 210), bn);
            } else {
                const char *warn = (bandplan_active() == BP_USER && bandplan_user_n() == 0) ? "EMPTY" : "NO CLOCK";
                text(B_PLAN.x + (B_PLAN.w - text_w(warn, 1)) / 2, B_PLAN.y + 28, 1,
                     RGB565(255, 200, 90), warn);
            }
        }
    }
    for (int i = 0; i < N_MODE_ROW; i++) {
        int m = MODE_ROW[i];
        box_t b = { MODE_BTN_X(i), MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H };
        draw_box(&b, dsp_mode_name(m), 2, (m == s_mode) ? mode_color(m) : COL_BTN);
    }
    draw_box(&B_HF, "HF", 2, COL_BTN);
    draw_box(&B_VHF, "VHF", 2, COL_BTN);
    draw_box(&B_UHF, "UHF", 2, COL_BTN);
    { /* PSK: tekst gore, indikator veze dolje */
    int st = psk_status();

    rect(B_PSK.x, B_PSK.y, B_PSK.w, B_PSK.h, COL_BTN);
    rect(B_PSK.x, B_PSK.y, B_PSK.w, 2, COL_BTN_HI);

    /* Natpis PSK u gornjem dijelu tipke */
    text(B_PSK.x + 18, B_PSK.y + 3, 2, COL_TEXT, "PSK");

    /* Mala kućica centrirana ispod natpisa */
    int cx = B_PSK.x + (B_PSK.w - 14) / 2;
    int cy = B_PSK.y + 24;

    rect(cx, cy, 14, 14, COL_TEXT);

    if (st) {
        rect(cx + 3, cy + 3, 8, 8,
             st == 2 ? RGB565(40, 190, 90) :
                       RGB565(220, 170, 40));
    }
}
    draw_box(&B_LORARX, "LoRaRX", 1, lora_rx_enabled() ? RGB565(24,110,70) : COL_BTN);
    draw_box(&B_HORUS, "HORUS", 1, (s_mode == DSP_MODE_HORUS) ? COL_BTN_HI : COL_BTN);
    draw_box(&B_LORA, "L-APRS", 1, lora_aprs_enabled() ? RGB565(24,110,70) : COL_BTN);
    draw_box(&B_APRS, "APRS", 1, (s_mode == DSP_MODE_APRS) ? RGB565(24,110,70) : COL_BTN);

    char v[8];
    text(530, 212, 2, COL_TEXT, "VOL");
    draw_box(&B_VDN, "-", 3, COL_BTN);
    draw_box(&B_VUP, "+", 3, COL_BTN);
    snprintf(v, sizeof v, "%d", s_volume);
    text(520, 238, 3, COL_LINE, v);

    text(530, 286, 2, COL_TEXT, "SQL");
    draw_box(&B_SDN, "-", 3, COL_BTN);
    draw_box(&B_SUP, "+", 3, COL_BTN);
    snprintf(v, sizeof v, "%d", s_squelch);
    text(516, 312, 3, COL_LINE, v);
}
/* ---- actions ------------------------------------------------------------- */

static bool retune(int64_t hz)
{
    if (hz < FREQ_MIN) hz = FREQ_MIN;
    if (hz > FREQ_MAX) hz = FREQ_MAX;
    int64_t old = s_center;
    int rc = sdr_set_center_freq((uint32_t)hz);
    if (rc < 0) {
        ESP_LOGE(TAG, "RTL-SDR retune FAILED: requested %lld Hz (rc=%d)", (long long)hz, rc);
        return false;
    }
    s_center = (uint32_t)hz;
    /* Moved off the current FT8/WSPR sub-band (>5 kHz): drop the stale decode
     * list, the stations below the new dial are different ones. Small
     * within-band nudges keep the list. */
    int64_t d = hz > old ? hz - old : old - hz;
    if (d > 5000) {
        if (s_mode == DSP_MODE_FT8) ft8_reset();
        else if (s_mode == DSP_MODE_WSPR) wspr_reset();
    }
    return true;
}

/* Switch the RTL2832 sample rate and tell the DSP (both are applied live). */
static void set_rate(uint32_t hz)
{
    if (hz == 0 || hz == s_rate_cur) return;
    if (sdr_set_sample_rate(hz)) {
        dsp_set_sample_rate(hz);
        s_rate_cur = hz;
    }
}

static void set_step_hz(uint32_t hz)
{
    for (int i = 0; i < N_STEPS; i++)
        if (STEPS[i] == hz) { s_step_i = i; return; }
}

/* FT8/WSPR dial-frequency tables now live in bandplan.c (shared with the scheduler);
 * the cursors below are just the manual-preset positions. */
static int s_ft8_i = 3; /* 20 m */
static int s_wspr_i = 5; /* 20 m */
/* FT4 USB dial frequencies. Repeated FT4 button presses cycle this list. */
static const uint32_t s_ft4_freqs[] = {
    3575000u, 7047500u, 10140000u, 14080000u, 18104000u,
    21140000u, 24919000u, 28180000u, 50318000u
};
static int s_ft4_i = 3; /* 20 m */

/* Digital-mode button: narrow 96 kHz view, 100 Hz step; pressed again = next frequency of the
 * list, otherwise keep the dial if it is already on one of the frequencies, else jump to the
 * last used one. */
static void digital_preset(const uint32_t *list, int n, int *idx, bool again)
{
    set_rate(288000);
    set_step_hz(100);
    if (n <= 0 || !list) return;
    if (again) {
        int next = (*idx + 1) % n;
        if (retune(list[next])) *idx = next;
        return;
    }
    for (int i = 0; i < n; i++) {
        int64_t d = (int64_t)s_center - (int64_t)list[i];
        if (d > -3000 && d < 3000) { *idx = i; return; }
    }
    retune(list[*idx]);
}

/* Retune so the strongest peak in the current spectrum sits on the centre line.
 * Uses the power-weighted centroid of the peak (bin spacing = span / bins: 1.152 MHz or, in HF mode, 96 kHz). */
static void snap_to_peak(void)
{
    const float *spec = dsp_spectrum();
    int bins = dsp_spectrum_bins();
    if (!spec || bins < 32) return;
    double sum = 0;
    float mx = -1e9f;
    int mi = -1;
    for (int i = 0; i < bins; i++) {
        sum += spec[i];
        if (abs(i - bins / 2) <= 3) continue; /* skip the RTL DC spike */
        if (spec[i] > mx) { mx = spec[i]; mi = i; }
    }
    if (mi < 0 || mx < (float)(sum / bins) + 8.0f) return; /* no clear peak */
    double pw = 0, pwi = 0;
    for (int i = mi - 5; i <= mi + 5; i++) {
        if (i < 0 || i >= bins) continue;
        double p = pow(10.0, (spec[i] - mx) / 10.0);
        if (p < 0.1) continue; /* only the top 10 dB */
        pw += p;
        pwi += p * i;
    }
    double c = pw > 0 ? pwi / pw : mi;
    int64_t off = (int64_t)llround((c - bins / 2) * ((double)dsp_span_hz() / bins));
    retune((int64_t)s_center + off);
}

/* Sweep in overlapping 1 MHz FFT windows. Find a peak, retune it to the
 * discriminator centre, then demand a CRC-valid RS41 frame before locking.
 * Avoid interpreting old decoder data as a new lock by comparing seq values. */
static void rs_scan_next(uint32_t now)
{
    const uint32_t first = 400500000u;
    const uint32_t step = 900000u;
    uint32_t f = first + (uint32_t)s_rs_scan_step * step;
    if (f > 405500000u) { s_rs_scan_step = 0; f = first; }
    s_rs_scan_step++;
    retune(f);
    rs41_reset();
    s_rs_scan_since = now;
    s_rs_scan = RS_SCAN_SWEEP;
    ESP_LOGI("rs41_scan", "Sweep %.3f MHz", f / 1e6);
}
static void mode_button(int m);
static void rs_scan_start(void)
{
    if (lora_aprs_enabled()) return;
    if (s_mode != DSP_MODE_RS41) mode_button(DSP_MODE_RS41);
    s_view_span = 0;
    s_rs_scan_step = 0;
    rs41_info_t r;
    s_rs_scan_seq = rs41_get_info(&r) ? r.seq : 0;
    rs_scan_next(pdTICKS_TO_MS(xTaskGetTickCount()));
    ESP_LOGI("rs41_scan", "Auto scan 400-406 MHz started");
}
static bool rs_scan_tick(void)
{
    if (s_rs_scan == RS_SCAN_OFF) return false;
    if (s_mode != DSP_MODE_RS41 || lora_aprs_enabled()) {
        rs_scan_stop(); return true;
    }
    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
    rs41_info_t r;
    if (rs41_get_info(&r) && r.seq != s_rs_scan_seq) {
        s_rs_scan_seq = r.seq;
        if (r.have_id && r.blk_ok) {
            if (s_rs_scan != RS_SCAN_LOCKED) ESP_LOGI("rs41_scan", "LOCK %s @ %.3f MHz", r.id, s_center / 1e6);
            s_rs_scan = RS_SCAN_LOCKED;
            s_rs_scan_last_frame = now;
            return true;
        }
    }
    if (s_rs_scan == RS_SCAN_LOCKED) {
        if (now - s_rs_scan_last_frame >= 60000u) {
            ESP_LOGI("rs41_scan", "No valid frame for 60s, resuming");
            rs_scan_next(now);
            return true;
        }
        return false;
    }
    if (s_rs_scan == RS_SCAN_VERIFY) {
        if (now - s_rs_scan_since >= 4500u) {
            rs_scan_next(now);
            return true;
        }
        return false;
    }
    /* Allow FFT to settle after each tune; inspect at ~1.2 seconds. */
    if (now - s_rs_scan_since < 1200u) return false;
    const float *spec = dsp_spectrum();
    int bins = dsp_spectrum_bins();
    if (spec && bins > 64) {
        double avg = 0;
        for (int i = 0; i < bins; i++) avg += spec[i];
        avg /= bins;
        int peak = -1;
        float max = (float)avg + 13.0f;
        for (int i = bins / 10; i < bins * 9 / 10; i++) {
            if (abs(i - bins / 2) < 10) continue; /* RTL DC spike */
            if (spec[i] > max) { max = spec[i]; peak = i; }
        }
        if (peak >= 0) {
            int64_t offset = (int64_t)llround((peak - bins / 2) *
                                                  ((double)dsp_span_hz() / bins));
            retune((int64_t)s_center + offset);
            rs41_reset();
            s_rs_scan = RS_SCAN_VERIFY;
            s_rs_scan_since = now;
            ESP_LOGI("rs41_scan", "Candidate %.3f MHz, checking RS41 CRC", s_center / 1e6);
            return true;
        }
    }
    rs_scan_next(now);
    return true;
}

static void set_mode(int mode)
{
    if (mode < 0 || mode >= DSP_MODE_COUNT) mode = DSP_MODE_WFM;
    if (is_panel_mode(s_mode) && !is_panel_mode(mode)) {
        rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0)); /* clear the text panel */
        s_panel_mode = -1;
    }
    s_mode = mode;
    dsp_set_mode(mode);
}

/* Band-plan hook (bandplan.c): retune to the scheduled slot. Runs in the UI
 * task context; switches mode + rate like a digital-mode preset. */
static char s_plan_label[24];
static void plan_apply(int mode, uint32_t freq)
{
    set_rate(288000);
    set_step_hz(100);
    set_mode(mode);
    retune((int64_t)freq);
    char bn[8];
    snprintf(s_plan_label, sizeof s_plan_label, "PLAN %s", bandplan_band_name(freq, bn, sizeof bn));
    s_label = s_plan_label;
}

/* A mode button was pressed: select the mode and its sensible defaults. */
static void mode_button(int m)
{
    if (m != DSP_MODE_RS41) rs_scan_stop();
    const bool again = (s_mode == m); /* same mode pressed twice */
    set_mode(m);
    s_label = "MANUAL";
    /* sensible default step: 100 Hz for SSB/CW (SSB needs ~50 Hz accuracy), 25 kHz otherwise */
    if (m == DSP_MODE_RS41) {
        s_view_span = 0; /* Reset zoom on RS41 selection. */
        set_rate(1152000); /* need the full 1.15 MHz view to find the sonde */
        s_step_i = 2; /* 1 kHz */
        /* sondes live at 400-406 MHz; jump there if we're elsewhere */
        if (s_center < 400000000u || s_center > 406000000u) retune(403000000);
    } else if (m == DSP_MODE_APRS) {
        s_view_span = 0; /* Reset zoom on APRS selection. */
        set_rate(288000);
        s_step_i = 2;
        retune(144800000u);
        s_label = "APRS 1200 VHF";
    } else if (m == DSP_MODE_HORUS) {
        s_view_span = 0; /* Reset zoom on HORUS selection. */
        set_rate(1152000);
        s_step_i = 2;
        /* Always force the user's Horus V1 frequency on button press.
         * This also allows recovery from saved/manual/scheduled frequencies. */
        retune(437600000u);
        ESP_LOGI(TAG, "HORUS selected: requested 437600000 Hz, SDR retune called");
        s_label = "HORUS 437.600";
    } else if (m == DSP_MODE_FT8) {
        int n;
        const uint32_t *f = bandplan_ft8_freqs(&n);
        digital_preset(f, n, &s_ft8_i, again);
        s_label = "FT8";
    } else if (m == DSP_MODE_FT4) {
        s_qso_has_selected = false;
        s_cq_active = false;
        s_auto = false;
        ft4_set_qso_focus(NULL, 0);
        digital_preset(s_ft4_freqs,
                       (int)(sizeof(s_ft4_freqs) / sizeof(s_ft4_freqs[0])),
                       &s_ft4_i, again);
        s_label = "FT4";
    } else if (m == DSP_MODE_WSPR) {
        int n;
        const uint32_t *f = bandplan_wspr_freqs(&n);
        digital_preset(f, n, &s_wspr_i, again);
        s_label = "WSPR";
    } else if (m >= DSP_MODE_USB) s_step_i = 0;
    else if (STEPS[s_step_i] < 1000) s_step_i = 6;
}

static void apply_squelch(void)
{
    dsp_set_squelch_db(s_squelch <= -60 ? -200.0f : (float)s_squelch + SQ_OFFSET);
}

/* ---- per-band memory -------------------------------------------------------
 * Each band button remembers where you were: frequency, mode, tuning step and
 * sample rate. Leaving a band (pressing another band button) saves its state;
 * coming back restores it. Pressing the button of the band you are already in
 * cycles through that band's presets (= back to the start frequency). */
enum { BAND_HF, BAND_VHF, BAND_UHF, N_BANDS };

typedef struct {
    bool valid;
    uint32_t hz, rate;
    int mode, step_i;
} band_mem_t;

static band_mem_t s_mem[N_BANDS];

static int band_of(uint32_t hz)
{
    if (hz < 30000000u) return BAND_HF;
    if (hz < 300000000u) return BAND_VHF;
    return BAND_UHF;
}

static void band_save(void)
{
    band_mem_t *m = &s_mem[band_of(s_center)];
    m->valid = true;
    m->hz = s_center;
    m->rate = s_rate_cur;
    m->mode = s_mode;
    m->step_i = s_step_i;
}

static void select_bookmark(const bookmark_t *b);

static void band_press(int band, const bookmark_t *list, int n, int *cursor, const char *last_label)
{
    rs_scan_stop();
    /* Restore the full waterfall/spectrum width whenever a band button is pressed.
     * Set to zero so view_span_hz() adopts the DSP span after set_rate(). */
    s_view_span = 0;
    if (band_of(s_center) == band) { /* already in this band: next preset */
        *cursor = (*cursor + 1) % n;
        select_bookmark(&list[*cursor]);
        return;
    }
    band_save(); /* remember where we are leaving */
    const band_mem_t *m = &s_mem[band];
    if (m->valid) {
        set_rate(m->rate);
        retune(m->hz);
        set_mode(m->mode);
        s_step_i = m->step_i;
        s_label = last_label;
    } else { /* first visit: the band's start preset */
        *cursor = 0;
        select_bookmark(&list[0]);
    }
}

static void select_bookmark(const bookmark_t *b)
{
    set_rate(b->rate); /* before retuning: the new rate re-applies the tuner bandwidth */
    retune(b->hz);
    set_mode(b->mode);
    if (b->step_hz) set_step_hz(b->step_hz);
    s_label = b->name;
}

static bool hit(const box_t *b, int x, int y)
{
    return x >= b->x && x < b->x + b->w && y >= b->y && y < b->y + b->h;
}

/* ---- screensaver (display off) ---------------------------------------------
 * After the configured idle time the backlight is turned off (the SDR keeps
 * running; nothing is rendered while the screen is off). Any touch turns the
 * backlight back on and is consumed — waking the display never presses a
 * button. The delay is set from a SCREEN OFF row in the PLAN menu and
 * persisted in NVS ("ui"/"scroff"). */

#define UI_BRIGHTNESS 80 /* must match main.c */

static const uint32_t SCR_OFF_MS[]  = {0, 30000, 60000, 120000, 300000, 600000};
static const char *const SCR_OFF_LBL[] = {"OFF", "30 S", "1 MIN", "2 MIN", "5 MIN", "10 MIN"};
#define N_SCR_OFF ((int)(sizeof SCR_OFF_MS / sizeof SCR_OFF_MS[0]))

static int s_scr_off_i = 0;                 /* selected delay index (0 = disabled) */
static bool s_scr_off = false;              /* display currently off */
static uint32_t s_last_touch_ms;            /* last activity (touch), ms ticks */
static bool s_touch_beep = true;
static void touch_beep_save(void)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "touchbeep", s_touch_beep ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}
static void touch_beep_load(void)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 1;
        if (nvs_get_u8(h, "touchbeep", &v) == ESP_OK) s_touch_beep = v != 0;
        nvs_close(h);
    }
}


static void scr_off_save(void)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "scroff", (uint8_t)s_scr_off_i);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void scr_off_load(void)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(h, "scroff", &v) == ESP_OK && v < N_SCR_OFF) s_scr_off_i = v;
        nvs_close(h);
    }
}

static void scr_off_kick(void) { s_last_touch_ms = pdTICKS_TO_MS(xTaskGetTickCount()); }

static void scr_off_set(bool off)
{
    if (off == s_scr_off) return;
    s_scr_off = off;
    bsp_tab5_display_set_brightness(off ? 0 : UI_BRIGHTNESS);
}

/* ---- band-plan menu --------------------------------------------------------
 * s_menu: 0 = closed, 1 = plan list, 2 = USER-plan editor. While a menu is
 * open it owns the whole screen and all touches; the normal controls are
 * unreachable until it is closed. */

static void menu_close(void);
/* s_menu declared above QSO panel */
/* RS41 archive: menu 3=file list, 4=CSV contents. Only the UI task reads files. */
#define RS41_LOG_PAGE 8
#define RS41_LOG_MAX 128
static char s_log_names[RS41_LOG_MAX][64];
static int s_log_count, s_log_page, s_log_selected, s_log_lines_page;
static int s_log_dir_errno;
static void rs41_log_refresh(void)
{
    s_log_count = 0;
    s_log_dir_errno = 0;
    DIR *d = opendir("/sdcard/RS41");
    if (!d) {
        s_log_dir_errno = errno;
        ESP_LOGE("rs41_ui", "Cannot open /sdcard/RS41: errno=%d", errno);
        return;
    }
    struct dirent *e;
    while ((e = readdir(d)) && s_log_count < RS41_LOG_MAX) {
        const char *name = e->d_name;
        size_t n = strlen(name);
        ESP_LOGI("rs41_ui", "SD entry: '%s' (len=%u)", name, (unsigned)n);
        if (n < 5) continue;
        /* FAT directory entries can have an upper-case extension. */
        if (strcasecmp(name + n - 4, ".csv") != 0) continue;
        if (n >= sizeof(s_log_names[0])) continue;
        snprintf(s_log_names[s_log_count++], sizeof(s_log_names[0]),
                 "%.*s", (int)sizeof(s_log_names[0]) - 1, name);
    }
    closedir(d);
    ESP_LOGI("rs41_ui", "RS41 CSV files found: %d", s_log_count);
    for (int i = 1; i < s_log_count; i++) {
        char tmp[64]; strcpy(tmp, s_log_names[i]);
        int j = i;
        while (j > 0 && strcmp(s_log_names[j-1], tmp) > 0) {
            strcpy(s_log_names[j], s_log_names[j-1]); j--;
        }
        strcpy(s_log_names[j], tmp);
    }
    if (s_log_page * RS41_LOG_PAGE >= s_log_count) s_log_page = 0;
}
static void rs41_log_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10, 14, 30));
    text(12, 8, 2, COL_ACCENT, s_menu == 3 ? "RS41 SD LOGS" : "RS41 CSV VIEW");
    rect(534, 4, 96, 31, COL_BTN);
    text(551, 12, 2, COL_TEXT, "BACK");
    if (!rs41_log_ready()) {
        text(20, 75, 2, COL_TEXT, "SD CARD NOT MOUNTED");
        return;
    }
    if (s_menu == 3) {
        char msg[80];
        snprintf(msg, sizeof(msg), "FILES %d  PAGE %d/%d", s_log_count, s_log_page+1,
                 s_log_count ? (s_log_count+RS41_LOG_PAGE-1)/RS41_LOG_PAGE : 1);
        text(12, 43, 1, COL_LINE, msg);
        if (!s_log_count) {
            if (s_log_dir_errno) {
                text(24, 100, 2, COL_TEXT, "SD DIRECTORY ERROR");
                char eb[40];
                snprintf(eb, sizeof eb, "errno=%d", s_log_dir_errno);
                text(24, 123, 1, COL_TEXT, eb);
            } else {
                text(24, 100, 2, COL_TEXT, "NO RS41 CSV FILES YET");
            }
        }
        for (int i = 0; i < RS41_LOG_PAGE; i++) {
            int idx = s_log_page * RS41_LOG_PAGE + i;
            if (idx >= s_log_count) break;
            int y = 64 + i * 31;
            rect(14, y, 610, 28, RGB565(28, 45, 67));
            text(25, y+7, 2, COL_TEXT, s_log_names[idx]);
            char path[112]; struct stat st;
            snprintf(path, sizeof path, "/sdcard/RS41/%s", s_log_names[idx]);
            if (stat(path, &st) == 0) {
                char sz[40]; snprintf(sz, sizeof sz, "%lu B", (unsigned long)st.st_size);
                text(522, y+10, 1, COL_LINE, sz);
            }
        }
    } else {
        if (s_log_selected < 0 || s_log_selected >= s_log_count) return;
        text(12, 43, 2, COL_LINE, s_log_names[s_log_selected]);
        char path[112]; snprintf(path, sizeof path, "/sdcard/RS41/%s", s_log_names[s_log_selected]);
        FILE *f = fopen(path, "r");
        if (!f) { text(20, 100, 2, COL_TEXT, "CANNOT OPEN FILE"); return; }
        char line[256]; int row = 0, skip = s_log_lines_page * 10;
        /* Skip header, then page through CSV records. */
        fgets(line, sizeof line, f);
        while (skip-- > 0 && fgets(line, sizeof line, f)) {}
        while (row < 10 && fgets(line, sizeof line, f)) {
            char out[192];
            /* Existing RS41 CSV: serial, frame, ..., has_position,
             * latitude, longitude, altitude_m, speed_ms, heading_deg,
             * climb_ms, satellites, rs_fixed, crc_blocks. */
            char *fields[18] = {0};
            int nf = 0;
            char *start = line;
            for (char *q = line; ; ++q) {
                if (*q == ',' || *q == '\r' || *q == '\n' || *q == 0) {
                    char end = *q;
                    *q = 0;
                    if (nf < 18) fields[nf++] = start;
                    start = q + 1;
                    if (end != ',') break;
                }
            }
            if (nf >= 15 && fields[8][0] == '1' && fields[9][0] && fields[10][0]) {
                /* RS41 vh is horizontal wind speed in m/s; vv is vertical
                 * speed in m/s (negative during descent). */
                float wind_kmh = strtof(fields[12], NULL) * 3.6f;
                float vv = strtof(fields[14], NULL);
                float descent_ms = vv < 0.0f ? -vv : 0.0f;
                snprintf(out, sizeof out,
                         "%.8s FRM: %.6s LAT: %.10s LON: %.11s A/D: %.1fm/s WIND: %.1fkm/h ALT: %.8sm",
                         fields[0], fields[1], fields[9], fields[10],
                         descent_ms, wind_kmh, fields[11]);
            } else if (nf >= 12) {
                snprintf(out, sizeof out, "%.8s FRM %.6s GPS - ALT -",
                         fields[0], fields[1]);
            } else {
                snprintf(out, sizeof out, "INVALID CSV RECORD");
            }
            text(12, 72 + row*23, 1, COL_TEXT, out);
            row++;
        }
        fclose(f);
        if (!row) text(20, 105, 2, COL_TEXT, "NO MORE RECORDS");
        char pg[40]; snprintf(pg, sizeof pg, "RECORDS %d-%d", s_log_lines_page*10+1, s_log_lines_page*10+row);
        text(14, 43+247, 1, COL_LINE, pg);
    }
    rect(12, 313, 150, 36, COL_BTN); text(29, 323, 2, COL_TEXT, "< PREV");
    rect(477, 313, 150, 36, COL_BTN); text(499, 323, 2, COL_TEXT, "NEXT >");
}
static bool rs41_log_touch(int x, int y)
{
    if (y < 40 && x > 520) {
        if (s_menu == 4) { s_menu = 3; rs41_log_refresh(); }
        else menu_close();
        return true;
    }
    if (y >= 310) {
        int delta = x < 175 ? -1 : x > 460 ? 1 : 0;
        if (s_menu == 3) {
            int maxpage = s_log_count ? (s_log_count-1)/RS41_LOG_PAGE : 0;
            s_log_page += delta;
            if (s_log_page < 0) s_log_page = 0;
            if (s_log_page > maxpage) s_log_page = maxpage;
        } else {
            s_log_lines_page += delta;
            if (s_log_lines_page < 0) s_log_lines_page = 0;
        }
        return true;
    }
    if (s_menu == 3 && y >= 64 && y < 64+RS41_LOG_PAGE*31) {
        int idx = s_log_page * RS41_LOG_PAGE + (y-64)/31;
        if (idx < s_log_count) { s_log_selected = idx; s_log_lines_page = 0; s_menu = 4; }
        return true;
    }
    return false;
}


/* PLAN menu: OFF, USER PLAN, EDIT USER PLAN, SCREEN OFF, SETTINGS, CLOSE. */
#define M1_ROWS      9
static const int s_plan_menu_actions[M1_ROWS] = {BP_OFF, BP_USER, -1, -2, -3, -5, -6, -7, -4};
#define M1_ROW_Y0    48
#define M1_ROW_PITCH 30
#define M1_ROW_H     27
#define M1_ROW_X     16
#define M1_ROW_W     608

/* USER-plan editor rows: one per slot + ADD SLOT + BACK. Tap zones per row:
 * hour (24 h wrap per tap), mode (FT8<->WSPR), band (next band of the mode),
 * X = delete the row. */
#define E_ROW_Y0    48
#define E_ROW_PITCH 22
#define E_HOUR_X    24
#define E_MODE_X    130
#define E_BAND_X    250
#define E_DEL_X     560

/* Si5351 calibration page (menu 10). Units: 0.1 ppm. */
static int32_t s_si_cal_edit;
static int32_t s_si_cal_saved;
static bool s_si_cal_error;
static bool s_tone_error;
/* Extra FT8 TX selection API; existing si5351.h is unchanged. */
extern esp_err_t si5351_ft8_set_base_cHz(uint64_t base_chz);
extern uint64_t si5351_ft8_get_base_cHz(void);
typedef struct { const char *band; uint32_t dial_hz; } ft8_tx_band_t;
static const ft8_tx_band_t s_ft8_tx_bands[] = {
    {"80M", 3573000}, {"40M", 7074000}, {"30M", 10136000},
    {"20M", 14074000}, {"17M", 18100000}, {"15M", 21074000},
    {"12M", 24915000}, {"10M", 28074000}, {"TEST", 0}
};
#define FT8_TX_BAND_COUNT 9
static int s_ft8_tx_band = 8; /* original 10 MHz test */
static uint64_t ft8_tx_base_for_band(int i)
{
    return s_ft8_tx_bands[i].dial_hz ?
        ((uint64_t)s_ft8_tx_bands[i].dial_hz + (uint64_t)s_cq_tx_audio_hz) * 100ULL : 1000000000ULL;
}

static void tone_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10,14,30));
    text(16, 8, 2, COL_ACCENT, "FT8 TX BAND");
    char cq_heading[96];
    snprintf(cq_heading, sizeof cq_heading, "CQ %s %s  -  79 SYMBOLS / UTC 15S", station_call(), station_grid());
    text(16, 31, 1, COL_TEXT, cq_heading);
    bool running = si5351_tone_test_running();
    for (int i = 0; i < FT8_TX_BAND_COUNT; ++i) {
        box_t r = {16 + (i % 3) * 208, 55 + (i / 3) * 48, 192, 42};
        draw_box(&r, s_ft8_tx_bands[i].band, 2,
                 i == s_ft8_tx_band ? RGB565(25, 110, 75) : COL_BTN);
    }
    char b[96];
    uint64_t chz = ft8_tx_base_for_band(s_ft8_tx_band);
    snprintf(b, sizeof b, "RF %.6f MHz  (%s)", (double)chz / 100000000.0,
             s_ft8_tx_band == 8 ? "10 MHZ TEST" : "USB DIAL + CQ TX AUDIO");
    text(16, 202, 1, COL_LINE, b);
    box_t start = {16, 226, 190, 55};
    box_t stop = {225, 226, 190, 55};
    box_t back = {434, 226, 190, 55};
    draw_box(&start, "START", 2, COL_BTN);
    draw_box(&stop, "STOP", 2, COL_BTN);
    draw_box(&back, "BACK", 2, COL_BTN);
    int tone = si5351_tone_test_index();
    snprintf(b, sizeof b, "STATUS %s%s", running ? "RUNNING" : "STOPPED",
             running && tone >= 0 ? "  TX ACTIVE" : "");
    text(16, 292, 1, running ? COL_ACCENT : COL_TEXT, b);
    if (running && tone >= 0) {
        snprintf(b, sizeof b, "SYMBOL %d / 79", tone + 1);
        text(270, 292, 1, COL_LINE, b);
    }
    text(16, 320, 1, s_tone_error ? RGB565(255,100,100) : COL_TEXT,
         s_tone_error ? "TX ERROR - SEE MONITOR" : "TEST ONLY - NO PTT - NO RF AMPLIFIER");
}
static bool tone_touch(int x, int y)
{
    if (y >= 55 && y < 193) {
        int row = (y - 55) / 48;
        int col = (x - 16) / 208;
        if (x >= 16 && col >= 0 && col < 3 && (x - 16) % 208 < 192 &&
            (y - 55) % 48 < 42 && row >= 0 && row < 3 &&
            !si5351_tone_test_running()) {
            int idx = row * 3 + col;
            if (si5351_ft8_set_base_cHz(ft8_tx_base_for_band(idx)) == ESP_OK) {
                s_ft8_tx_band = idx;
                s_tone_error = false;
            } else s_tone_error = true;
        }
        return true;
    }
    if (y >= 226 && y < 281) {
        if (x >= 16 && x < 206) {
            if (!si5351_tone_test_running()) {
                esp_err_t err = si5351_ft8_set_base_cHz(ft8_tx_base_for_band(s_ft8_tx_band));
                if (err == ESP_OK) err = si5351_tone_test_start();
                s_tone_error = err != ESP_OK;
            }
        } else if (x >= 225 && x < 415) {
            si5351_tone_test_stop();
        } else if (x >= 434 && x < 624) {
            si5351_tone_test_stop();
            s_menu = 1;
        }
    }
    return true;
}
static void si_cal_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10,14,30));
    text(16, 12, 2, COL_ACCENT, "SI5351 CAL");
    text(16, 50, 2, COL_TEXT, "CLK0  10.000000 MHz");
    char b[64];
    snprintf(b, sizeof b, "CORRECTION: %+.1f ppm", s_si_cal_edit / 10.0);
    text(16, 90, 2, COL_LINE, b);
    snprintf(b, sizeof b, "SAVED: %+.1f ppm", s_si_cal_saved / 10.0);
    text(16, 122, 1, COL_TEXT, b);
    static const char *names[4] = {"-1", "-0.1", "+0.1", "+1"};
    for (int i = 0; i < 4; ++i) {
        box_t r = {16 + i * 156, 163, 144, 52};
        draw_box(&r, names[i], 2, COL_BTN);
    }
    box_t save = {16, 247, 290, 52};
    box_t back = {334, 247, 290, 52};
    draw_box(&save, "SAVE", 2, COL_BTN);
    draw_box(&back, "BACK", 2, COL_BTN);
    text(16, 317, 1, s_si_cal_error ? RGB565(255,100,100) : COL_TEXT,
         s_si_cal_error ? "SI5351 I2C/NVS ERROR" : "ADJUST LIVE - SAVE STORES IN NVS");
}
static bool si_cal_touch(int x, int y)
{
    if (y >= 163 && y < 215 && x >= 16 && x < 628) {
        int i = (x - 16) / 156;
        if (i >= 0 && i < 4 && (x - 16) % 156 < 144) {
            static const int delta[4] = {-10, -1, 1, 10};
            int32_t next = s_si_cal_edit + delta[i];
            if (next >= -1000 && next <= 1000) {
                esp_err_t err = si5351_apply_calibration(next);
                s_si_cal_error = (err != ESP_OK);
                if (err == ESP_OK) s_si_cal_edit = next;
            }
        }
        return true;
    }
    if (y >= 247 && y < 299) {
        if (x >= 16 && x < 306) {
            esp_err_t err = si5351_set_calibration(s_si_cal_edit);
            s_si_cal_error = (err != ESP_OK);
            if (err == ESP_OK) s_si_cal_saved = s_si_cal_edit;
            return true;
        }
        if (x >= 334 && x < 624) {
            if (s_si_cal_edit != s_si_cal_saved) {
                esp_err_t err = si5351_apply_calibration(s_si_cal_saved);
                if (err != ESP_OK) { s_si_cal_error = true; return true; }
            }
            s_menu = 1;
            return true;
        }
    }
    return true;
}

/* SETTINGS editor: menu 5 = fields, 6 = touch keyboard. */
static station_settings_t s_st_edit;
static int s_st_field;
static bool s_st_lower;
static bool s_st_saved;
static char *settings_field(void)
{
    switch (s_st_field) {
    case 0: return s_st_edit.ssid;
    case 1: return s_st_edit.password;
    case 2: return s_st_edit.callsign;
    default: return s_st_edit.grid;
    }
}
static size_t settings_capacity(void)
{
    switch (s_st_field) {
    case 0: return sizeof s_st_edit.ssid;
    case 1: return sizeof s_st_edit.password;
    case 2: return sizeof s_st_edit.callsign;
    default: return sizeof s_st_edit.grid;
    }
}
static void settings_draw(void)
{
    rect(0,0,LW,LH,RGB565(10,14,30));
    if (s_menu == 5) {
        text(12,5,2,COL_ACCENT,"STATION SETTINGS");
        const char *labels[4] = {"WIFI SSID","WIFI PASSWORD","CALLSIGN","QTH LOCATOR"};
        const char *values[4] = {s_st_edit.ssid,"********",s_st_edit.callsign,s_st_edit.grid};
        for (int i=0;i<4;i++) {
            int y=34+i*45;
            rect(12,y,616,40,COL_BTN);
            text(22,y+6,2,COL_TEXT,labels[i]);
            text(290,y+6,2,RGB565(130,230,255),values[i]);
        }
        rect(12,220,294,40,RGB565(25,100,65));
        text(95,230,2,COL_TEXT,"SAVE");
        rect(334,220,294,40,COL_BTN);
        text(424,230,2,COL_TEXT,"BACK");
        rect(12,269,302,37,COL_BTN);
        rect(324,269,304,37,web_mirror_enabled() ? RGB565(25,100,65) : COL_BTN);
        char mode_label[48];
        snprintf(mode_label,sizeof mode_label,"WIFI: %s",wifi_net_mode_name());
        text(20,278,2,COL_TEXT,mode_label);
        text(337,278,2,COL_TEXT,web_mirror_enabled() ? "WEB: ON" : "WEB: OFF");
        char web_url[100];
        if (wifi_net_mode() != 0) {
            snprintf(web_url,sizeof web_url,"AP: %s  http://%s/",wifi_net_ap_ssid(),wifi_net_ap_ip());
            text(14,313,1,RGB565(130,230,255),web_url);
            snprintf(web_url,sizeof web_url,"AP PASS: %s",wifi_net_ap_password());
            text(14,331,1,COL_TEXT,web_url);
        } else if (wifi_net_connected()) {
            snprintf(web_url,sizeof web_url,"WEB: http://%s/",wifi_net_ip());
            text(14,318,1,RGB565(130,230,255),web_url);
        } else {
            text(14,318,1,COL_TEXT,"WEB: WIFI NOT CONNECTED");
        }
        if (s_st_saved) text(370,317,1,RGB565(150,255,160),"SAVED");
        return;
    }
    const char *labels[4]={"WIFI SSID","WIFI PASSWORD","CALLSIGN","QTH LOCATOR"};
    text(12,6,2,COL_ACCENT,labels[s_st_field]);
    char shown[72];
    const char *v=settings_field();
    /* Show actual characters while editing the WiFi password.
       The SETTINGS overview continues to mask the password. */
    snprintf(shown, sizeof shown, "%s", v);
    rect(12,36,616,44,COL_BTN);
    text(20,48,2,COL_TEXT,shown);
    const char *keys[4]={"1234567890","QWERTYUIOP","ASDFGHJKL-","ZXCVBNM._@"};
    for(int row=0;row<4;row++) for(int col=0;col<10;col++) {
        char ch=keys[row][col]; if(!ch)continue;
        int x=12+col*62,y=90+row*43;
        rect(x,y,58,39,COL_BTN);
        char label[2]={ch,0};
        /* Keep keyboard labels in the original legible uppercase font.
           CASE still controls the character inserted into the field. */
        text(x+19,y+10,2,COL_TEXT,label);
    }
    const char *special[5]={"CASE","SPACE","DEL","OK","CANCEL"};
    for(int i=0;i<5;i++) {
        int x=12+i*124;
        rect(x,268,120,43,i==0 && !s_st_lower ? RGB565(25,140,65) : (i==3 ? RGB565(25,100,65) : COL_BTN));
        text(x+12,281,2,COL_TEXT,special[i]);
    }
}
static bool settings_touch(int x,int y)
{
    if(s_menu==5) {
        if(y>=34 && y<214) {
            int i=(y-34)/45;
            if(i<4){s_st_field=i;s_st_lower=(i<=1);s_menu=6;}
            return true;
        }
        if(y>=220 && y<262) {
            if(x<318) {
                s_st_saved=station_settings_save(&s_st_edit);
                if(s_st_saved){ station_identity_reload(); wifi_net_apply_settings(); }
            } else {s_menu=1;}
            return true;
        }
        if(y>=269 && y<309) {
            if (x < 320) wifi_net_set_mode((wifi_net_mode()+1)%3);
            else web_mirror_set_enabled(!web_mirror_enabled());
            return true;
        }
        return true;
    }
    if(y>=90 && y<262) {
        int row=(y-90)/43,col=(x-12)/62;
        if(x>=12 && col>=0 && col<10 && row<4) {
            const char *keys[4]={"1234567890","QWERTYUIOP","ASDFGHJKL-","ZXCVBNM._@"};
            char ch=keys[row][col];
            if(ch) {
                if(s_st_lower && ch>='A' && ch<='Z')ch+=32;
                char *v=settings_field();size_t n=strlen(v);
                if(n+1<settings_capacity()){v[n]=ch;v[n+1]=0;}
            }
        }
        return true;
    }
    if(y>=268 && y<314 && x>=12) {
        int i=(x-12)/124;
        char *v=settings_field();size_t n=strlen(v);
        if(i==0)s_st_lower=!s_st_lower;
        if(i==1 && n+1<settings_capacity()){v[n]=' ';v[n+1]=0;}
        if(i==2 && n)v[n-1]=0;
        if(i==3 || i==4)s_menu=5;
        return true;
    }
    return true;
}

/* Generic LoRa RX screen (7), numeric keypad (8). APRS profile is separate. */
static lora_rx_config_t s_lr_edit;
static int s_lr_field;
static char s_lr_input[20];
static bool s_lr_saved;
static void lr_draw(void) {
    rect(0,0,LW,LH,RGB565(10,14,30));
    char b[128];
    if(s_menu==8) {
        const char *names[]={"FREQUENCY MHZ","SF (7-12)","BW KHZ","SYNC WORD HEX","CODING RATE (OFF, 5-8)"};
        text(12,8,2,COL_ACCENT,names[s_lr_field]);
        rect(12,40,616,45,COL_BTN);text(22,51,2,COL_TEXT,s_lr_input);
        const char *rows[]={"123","456","789",".0X"};
        for(int r=0;r<4;r++)for(int c=0;c<3;c++){
            int x=12+c*154,y=96+r*47;rect(x,y,148,42,COL_BTN);
            char ch[2]={rows[r][c],0};text(x+60,y+10,2,COL_TEXT,ch);
        }
        rect(476,96,152,43,COL_BTN);text(505,105,2,COL_TEXT,"DEL");
        if(s_lr_field==4){rect(476,144,152,42,COL_BTN);text(505,153,2,COL_TEXT,"OFF");}
        rect(476,192,152,90,RGB565(25,100,65));text(510,222,2,COL_TEXT,"OK");
        rect(12,292,616,32,COL_BTN);text(280,300,2,COL_TEXT,"BACK");
        return;
    }
    text(12,5,2,COL_ACCENT,"LORA RX - LIVE PACKETS");
    snprintf(b,sizeof b,"FREQ %.3f MHz   SF%u   BW %.2f kHz   SW 0x%02X",s_lr_edit.freq_hz/1000000.0,(unsigned)s_lr_edit.sf,s_lr_edit.bw_hz/1000.0,(unsigned)s_lr_edit.sync_word);
    text(12,29,1,COL_TEXT,b);
    const char *names[]={"FREQ","SF","BW","SYNC","CR"};
    for(int i=0;i<5;i++){int x=12+i*125;rect(x,49,119,35,COL_BTN);text(x+12,58,2,COL_TEXT,names[i]);}
    snprintf(b,sizeof b,"CR: %s",s_lr_edit.coding_rate==0?"AUTO":s_lr_edit.coding_rate==5?"4/5":s_lr_edit.coding_rate==6?"4/6":s_lr_edit.coding_rate==7?"4/7":"4/8");
    text(350,92,1,COL_TEXT,b);
    snprintf(b,sizeof b,"PACKETS: %lu",(unsigned long)lora_rx_packet_count());text(12,92,1,COL_ACCENT,b);
    lora_aprs_packet_t pk[LORA_RX_HISTORY];int n=lora_rx_packets(pk,LORA_RX_HISTORY);
    /* Larger font for received LoRa packets; fit lines above SAVE/BACK. */
    for(int i=0;i<n && i<8;i++){
        int y=112+i*22;
        snprintf(b,sizeof b,"SNR %.1f  %.30s",(double)pk[i].snr_db,pk[i].text);
        rs41_smooth_text(12,y,COL_TEXT,b);
    }
    rect(12,288,298,40,RGB565(25,100,65));text(100,298,2,COL_TEXT,"SAVE");
    rect(330,288,298,40,COL_BTN);text(420,298,2,COL_TEXT,"BACK");
    if(s_lr_saved)text(12,273,1,RGB565(150,255,160),"SAVED TO NVS");
}
static bool lr_touch(int x,int y){
    if(s_menu==7){
        if(y>=49 && y<87){int i=(x-12)/125;if(x>=12 && i>=0 && i<5){
            s_lr_field=i;
            if(i==0)snprintf(s_lr_input,sizeof s_lr_input,"%.3f",s_lr_edit.freq_hz/1000000.0);
            if(i==1)snprintf(s_lr_input,sizeof s_lr_input,"%u",(unsigned)s_lr_edit.sf);
            if(i==2)snprintf(s_lr_input,sizeof s_lr_input,"%.2f",s_lr_edit.bw_hz/1000.0);
            if(i==3)snprintf(s_lr_input,sizeof s_lr_input,"%02X",(unsigned)s_lr_edit.sync_word);
            if(i==4)snprintf(s_lr_input,sizeof s_lr_input,"%u",(unsigned)s_lr_edit.coding_rate);
            s_menu=8;
        }return true;}
        if(y>=288){
            if(x<320){s_lr_saved=lora_rx_save_config(&s_lr_edit);}
            else {lora_rx_enable(false);sdr_set_iq_discard(false);audio_set_volume(s_volume);menu_close();}
            return true;
        }
        return true;
    }
    if(y>=292){s_menu=7;return true;}
    if(x>=476 && y>=96 && y<139){size_t n=strlen(s_lr_input);if(n)s_lr_input[n-1]=0;return true;}
    if(x>=476 && y>=144 && y<186 && s_lr_field==4){snprintf(s_lr_input,sizeof s_lr_input,"OFF");return true;}
    if(x>=476 && y>=192 && y<282){
        char *end=NULL;
        if(s_lr_field==0){double v=strtod(s_lr_input,&end);if(end!=s_lr_input && !*end && v>=400 && v<=525)s_lr_edit.freq_hz=(uint32_t)(v*1000000+0.5);}
        if(s_lr_field==1){long v=strtol(s_lr_input,&end,10);if(end!=s_lr_input && !*end && v>=7 && v<=12)s_lr_edit.sf=(uint8_t)v;}
        if(s_lr_field==2){double v=strtod(s_lr_input,&end);const uint32_t widths[]={7800,10400,15600,20800,31250,41700,62500,125000,250000,500000};for(int i=0;i<10;i++)if(fabs(v*1000-widths[i])<80)s_lr_edit.bw_hz=widths[i];}
        if(s_lr_field==3){const char *t=s_lr_input;if(t[0]=='0' && (t[1]=='x'||t[1]=='X'))t+=2;unsigned long v=strtoul(t,&end,16);if(end!=t && !*end && v<=255)s_lr_edit.sync_word=(uint8_t)v;}
        if(s_lr_field==4){if(strcmp(s_lr_input,"OFF")==0 || strcmp(s_lr_input,"0")==0)s_lr_edit.coding_rate=0;
            else {long v=strtol(s_lr_input,&end,10);if(end!=s_lr_input && !*end && v>=5 && v<=8)s_lr_edit.coding_rate=(uint8_t)v;}}
        s_lr_saved=false;s_menu=7;return true;
    }
    if(y>=96 && y<284 && x>=12 && x<470){int r=(y-96)/47,c=(x-12)/154;if(r<4 && c<3){const char *rows[]={"123","456","789",".0X"};char ch=rows[r][c];size_t n=strlen(s_lr_input);if(n+1<sizeof s_lr_input){s_lr_input[n]=ch;s_lr_input[n+1]=0;}}return true;}
    return true;
}
static void menu_close(void)
{
    s_menu = 0;
    rect(0, 0, LW, LH, COL_BG);                 /* the menu drew over the whole screen, header + controls background included */
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0)); /* panel / waterfall area */
    s_panel_mode = -1;                          /* force the text-panel body to redraw */
}

static void menu_status(char *b, size_t n)
{
    if (!bandplan_time_ok()) {
        snprintf(b, n, "CLOCK NOT SET - PLANNER NEEDS WIFI + SNTP TIME SYNC");
        return;
    }
    int hh, mm;
    bandplan_now_hm(&hh, &mm);
    if (bandplan_active() == BP_OFF) {
        snprintf(b, n, "NOW %02d:%02d  PLANNER OFF", hh, mm);
        return;
    }
    bp_slot_t cur;
    if (bandplan_current(&cur)) {
        char bn[8];
        snprintf(b, n, "NOW %02d:%02d  %s %s UNTIL %02d:00", hh, mm,
                 dsp_mode_name(cur.mode), bandplan_band_name(bandplan_slot_freq(&cur), bn, sizeof bn),
                 bandplan_current_until());
    } else {
        snprintf(b, n, "NOW %02d:%02d  %s EMPTY", hh, mm, bandplan_name(bandplan_active()));
    }
}

static void plan_info(int plan, char *b, size_t n)
{
    switch (plan) {
    case BP_OFF:  snprintf(b, n, "NO AUTOMATIC BAND CHANGES"); break;
    case BP_FT8:  snprintf(b, n, "10 SLOTS  8 BANDS  80-10M"); break;
    case BP_WSPR: snprintf(b, n, "12 SLOTS  11 BANDS  160-6M"); break;
    default: {
        int c = bandplan_user_n();
        if (c) snprintf(b, n, "%d SLOTS", c);
        else snprintf(b, n, "EMPTY - EDIT TO ADD SLOTS");
    }
    }
}

static void menu_close(void);
/* Direct frequency entry from the large frequency display. */
static char s_freq_entry[20];
static void freq_entry_draw(void)
{
    rect(0, 0, LW, LH, RGB565(10, 14, 30));
    text(12, 8, 2, COL_ACCENT, "ENTER FREQUENCY (MHz)");
    rect(12, 40, 616, 44, COL_BTN);
    text(24, 49, 3, COL_TEXT, s_freq_entry[0] ? s_freq_entry : "_");
    static const char *keys[12] = {
        "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "DEL"
    };
    for (int i = 0; i < 12; ++i) {
        int x = 12 + (i % 3) * 208, y = 94 + (i / 3) * 48;
        rect(x, y, 198, 42, COL_BTN);
        text(x + 76, y + 10, 2, COL_TEXT, keys[i]);
    }
    rect(12, 294, 298, 50, COL_BTN);
    text(105, 308, 2, COL_TEXT, "CANCEL");
    rect(330, 294, 298, 50, RGB565(25, 100, 65));
    text(439, 308, 2, COL_TEXT, "TUNE");
}
static bool freq_entry_touch(int x, int y)
{
    if (y >= 294) {
        if (x >= 320 && s_freq_entry[0]) {
            char *end = NULL;
            double mhz = strtod(s_freq_entry, &end);
            if (end != s_freq_entry && *end == '\0' && isfinite(mhz) &&
                mhz >= (double)FREQ_MIN / 1e6 && mhz <= (double)FREQ_MAX / 1e6) {
                rs_scan_stop();
                retune((int64_t)llround(mhz * 1e6));
                s_label = "MANUAL";
            } else {
                ESP_LOGW(TAG, "Invalid frequency: %s", s_freq_entry);
                return true; /* keep editor open */
            }
        }
        s_menu = 0;
        menu_close();
        return true;
    }
    if (y >= 94 && y < 286 && x >= 12 && x < 636) {
        int col = (x - 12) / 208, row = (y - 94) / 48;
        if (col >= 0 && col < 3 && row >= 0 && row < 4) {
            static const char *keys[12] = {
                "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "DEL"
            };
            const char *k = keys[row * 3 + col];
            size_t len = strlen(s_freq_entry);
            if (row == 3 && col == 2) {
                if (len) s_freq_entry[len - 1] = '\0';
            } else if (len + 1 < sizeof s_freq_entry &&
                       (*k != '.' || !strchr(s_freq_entry, '.'))) {
                s_freq_entry[len] = *k;
                s_freq_entry[len + 1] = '\0';
            }
        }
    }
    return true;
}
static void draw_menu(void)
{
    rect(0, 0, LW, LH, RGB565(10, 14, 30));
    char b[96];
    if (s_menu == 1) {
        text(6, 4, 2, COL_ACCENT, "BAND PLAN");
        menu_status(b, sizeof b);
        text(6, 26, 1, COL_TEXT, b);
        for (int i = 0; i < M1_ROWS; i++) {
            int action = s_plan_menu_actions[i];
            box_t r = {M1_ROW_X, M1_ROW_Y0 + i * M1_ROW_PITCH, M1_ROW_W, M1_ROW_H};
            bool sel = action >= 0 && bandplan_active() == action;
            rect(r.x, r.y, r.w, r.h, sel ? RGB565(30, 110, 75) : COL_BTN);
            rect(r.x, r.y, r.w, 2, COL_BTN_HI);
            const char *label = action >= 0 ? bandplan_name(action) :
                (action == -1 ? "EDIT USER PLAN" : action == -2 ? "SCREEN OFF" :
                 action == -3 ? "SETTINGS" : action == -5 ? "TOUCH BEEP" :
                 action == -6 ? "SI5351 CAL" : action == -7 ? "FT8 TONE TEST" : "CLOSE");
            text(r.x + 12, r.y + 8, 2, COL_TEXT, label);
            if (action >= 0) {
                plan_info(action, b, sizeof b);
                text(r.x + 300, r.y + 11, 1, RGB565(170, 180, 200), b);
            } else if (action == -2) {
                text(r.x + 300, r.y + 11, 1, s_scr_off_i ? RGB565(190, 255, 210) : RGB565(170, 180, 200),
                     SCR_OFF_LBL[s_scr_off_i]);
            } else if (action == -5) {
                text(r.x + 300, r.y + 11, 1, s_touch_beep ? COL_ACCENT : COL_TEXT,
                     s_touch_beep ? "ON" : "OFF");
            }
        }
        return;
    }
    if (s_menu == 10) { si_cal_draw(); return; }
    if (s_menu == 11) { tone_draw(); return; }
    if (s_menu == 12) { qso_draw(); return; }
    if (s_menu == 13) { ft8_log_draw(); return; }
    if (s_menu == 9) { freq_entry_draw(); return; }
    if (s_menu == 5 || s_menu == 6) { settings_draw(); return; }
    if (s_menu == 7 || s_menu == 8) { lr_draw(); return; }
    /* USER-plan editor */
    text(6, 4, 2, COL_ACCENT, "EDIT USER PLAN");
    text(6, 26, 1, COL_TEXT, "TAP HOUR MODE OR BAND TO CHANGE    X = DELETE");
    int n = bandplan_user_n();
    for (int i = 0; i < n + 2; i++) {
        int ry = E_ROW_Y0 + i * E_ROW_PITCH;
        if (i < n) {
            bp_slot_t s = bandplan_user_get(i);
            char t[8], bn[8];
            snprintf(t, sizeof t, "%02d", s.hour);
            text(E_HOUR_X, ry + 3, 2, COL_LINE, t);
            text(E_MODE_X, ry + 3, 2, COL_TEXT, dsp_mode_name(s.mode));
            text(E_BAND_X, ry + 3, 2, RGB565(120, 220, 255), bandplan_band_name(bandplan_slot_freq(&s), bn, sizeof bn));
            text(E_DEL_X, ry + 3, 2, RGB565(255, 120, 120), "X");
            /* tap-zone underlines */
            rect(E_HOUR_X, ry + 19, 24, 1, RGB565(60, 70, 100));
            rect(E_MODE_X, ry + 19, 48, 1, RGB565(60, 70, 100));
            rect(E_BAND_X, ry + 19, 40, 1, RGB565(60, 70, 100));
        } else {
            rect(E_HOUR_X - 8, ry + 1, 130, 20, (i == n) ? RGB565(40, 80, 60) : RGB565(60, 60, 90));
            text(E_HOUR_X, ry + 3, 2, COL_TEXT, (i == n) ? "ADD SLOT" : "BACK");
        }
    }
}

static bool menu_touch(int x, int y)
{
    if (s_menu == 10) return si_cal_touch(x, y);
    if (s_menu == 11) return tone_touch(x, y);
    if (s_menu == 12) return qso_touch(x, y);
    if (s_menu == 13) return ft8_log_touch(x, y);
    if (s_menu == 9) return freq_entry_touch(x, y);
    if (s_menu == 5 || s_menu == 6) return settings_touch(x,y);
    if (s_menu == 7 || s_menu == 8) return lr_touch(x,y);
    if (s_menu == 3 || s_menu == 4) return rs41_log_touch(x, y);
    if (s_menu == 1) {
        for (int i = 0; i < M1_ROWS; i++) {
            box_t r = {M1_ROW_X, M1_ROW_Y0 + i * M1_ROW_PITCH, M1_ROW_W, M1_ROW_H};
            if (hit(&r, x, y)) {
                int action = s_plan_menu_actions[i];
                if (action >= 0) { bandplan_select(action); menu_close(); }
                else if (action == -1) s_menu = 2;
                else if (action == -2) { s_scr_off_i = (s_scr_off_i + 1) % N_SCR_OFF; scr_off_save(); }
                else if (action == -3) { station_settings_get(&s_st_edit); s_st_saved=false; s_menu=5; }
                else if (action == -5) { s_touch_beep = !s_touch_beep; touch_beep_save(); if (s_touch_beep) audio_touch_beep(); }
                else if (action == -7) { s_tone_error = false; s_menu = 11; }
                else if (action == -6) {
                    s_si_cal_edit = si5351_get_calibration();
                    s_si_cal_saved = s_si_cal_edit;
                    s_si_cal_error = false;
                    s_menu = 10;
                }
                else menu_close();
                return true;
            }
        }
        menu_close(); /* tap outside the list dismisses */
        return true;
    }
    /* editor */
    int n = bandplan_user_n();
    int row = (y - E_ROW_Y0) / E_ROW_PITCH;
    if (y < E_ROW_Y0 || row < 0 || row >= n + 2) return false;
    if (row < n) {
        bp_slot_t s = bandplan_user_get(row);
        int nb;
        if (x >= E_DEL_X) { bandplan_user_del(row); return true; }
        if (x < E_MODE_X) { s.hour = (s.hour + 1) % 24; bandplan_user_set(row, &s); return true; }
        if (x < E_BAND_X) { /* toggle the mode; keep the band index valid for the new table */
            s.mode = (s.mode == DSP_MODE_FT8) ? DSP_MODE_WSPR : DSP_MODE_FT8;
            if (s.mode == DSP_MODE_WSPR) bandplan_wspr_freqs(&nb); else bandplan_ft8_freqs(&nb);
            if (s.band >= nb) s.band = (uint8_t)(nb - 1);
            bandplan_user_set(row, &s);
            return true;
        }
        if (s.mode == DSP_MODE_WSPR) bandplan_wspr_freqs(&nb); else bandplan_ft8_freqs(&nb);
        s.band = (uint8_t)((s.band + 1) % nb);
        bandplan_user_set(row, &s);
        return true;
    }
    if (x < E_HOUR_X - 8 || x >= E_HOUR_X - 8 + 130) return false; /* ADD SLOT / BACK zone only */
    if (row == n) { /* ADD SLOT: next hour after the last slot, FT8 40 m default */
        bp_slot_t s = {8, DSP_MODE_FT8, 1};
        if (n > 0) { s = bandplan_user_get(n - 1); s.hour = (uint8_t)((s.hour + 1) % 24); s.band = 1; }
        bandplan_user_add(&s);
        return true;
    }
    s_menu = 1; /* BACK */
    return true;
}

static bool handle_touch(void)
{
    static bool prev = false, dragging = false;
    static int drag_x0, tap_y;
    static bool multi_gesture = false;
    static float pinch_start_dist = 0;
    static uint32_t pinch_start_span = 0;
    static bool tap_moved;
    static bool rs_hold = false, rs_hold_handled = false;
    static uint32_t rs_hold_since = 0;
    static uint32_t drag_f0;
    static int grace = 0;    /* ignore touches briefly after boot */
    static int run = 0;      /* consecutive raw-pressed samples (debounce) */
    bool changed = false;

    esp_lcd_touch_point_data_t p[2];
    int n = bsp_tab5_touch_read(p, 2);
    /* Web pointer uses the same touch pipeline. Physical touch takes priority. */
    int wx, wy; bool wd;
    if (n == 0 && web_mirror_pointer(&wx, &wy, &wd) && wd) {
        p[0].x = wy * SCALE;
        p[0].y = PANEL_H - wx * SCALE;
        n = 1;
    }

    /* ~1.2 s startup grace: the touch controller reports stray points right after
     * init that would otherwise "tap" a control. */
    if (grace < 150) { grace++; prev = false; run = 0; return false; }

    /* Debounce: a press must persist for 2 polls (~16 ms) before it counts,
     * rejecting single-sample glitches. */
    run = (n > 0) ? run + 1 : 0;
    /* Mode buttons need a responsive first touch: a short tap can last
     * only one UI poll. Keep the two-sample debounce everywhere else. */
    bool quick_mode_touch = false;
    if (n == 1 && s_menu == 0 && !s_scr_off) {
        int tx = (PANEL_H - p[0].y) / SCALE;
        int ty = p[0].x / SCALE;
        if (ty >= MODE_BTN_Y && ty < MODE_BTN_Y + MODE_BTN_H) {
            for (int i = 0; i < N_MODE_ROW; ++i) {
                box_t b = { MODE_BTN_X(i), MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H };
                if (hit(&b, tx, ty)) { quick_mode_touch = true; break; }
            }
        }
    }
    /* During an active FT8 AUTO QSO, any touch in the RX display is a
     * shortcut back to the QSO. Accept even a short, single-poll tap. */
    bool quick_qso_touch = false;
    if (n == 1 && s_menu == 0 && !s_scr_off &&
        s_mode == DSP_MODE_FT8 && s_auto && s_qso_has_selected &&
        !qso_ready_for_new_station() &&
        !lora_aprs_enabled()) {
        int tx = (PANEL_H - p[0].y) / SCALE;
        int ty = p[0].x / SCALE;
        quick_qso_touch = tx >= 0 && tx < LW && ty >= SPEC_Y0 && ty < CTRL_Y0;
    }
    /* FT8 decoded rows must respond to short taps as reliably as mode keys.
     * A touch in the list is never interpreted as a frequency gesture. */
    bool quick_ft8_row_touch = false;
    if (n == 1 && s_menu == 0 && !s_scr_off &&
        (s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) && !lora_aprs_enabled()) {
        int ty = p[0].x / SCALE;
        int tx = (PANEL_H - p[0].y) / SCALE;
        quick_ft8_row_touch = (ty >= WF_Y0 + 3 + PANEL_HDR_H &&
                              ty < WF_Y0 + 3 + PANEL_HDR_H + PANEL_ROWS * 10) ||
                             (tx >= FT8_BACK_X && tx < FT8_BACK_X + FT8_BACK_W &&
                              ty >= FT8_BACK_Y && ty < FT8_BACK_Y + FT8_BACK_H) ||
                             (tx >= FT8_FILTER_X && tx < FT8_FILTER_X + FT8_FILTER_W &&
                              ty >= FT8_FILTER_Y && ty < FT8_FILTER_Y + FT8_FILTER_H);
    }
    bool pressed = (run >= 2) || (run == 1 &&
                    (quick_mode_touch || quick_qso_touch || quick_ft8_row_touch));
    bool two = pressed && n >= 2;

    if (s_scr_off) { /* screensaver: the touch only wakes the display */
        if (pressed) { scr_off_set(false); scr_off_kick(); }
        prev = pressed;
        return false;
    }
    if (pressed) scr_off_kick();

    int lx = 0, ly = 0;
    if (pressed) {
        lx = (PANEL_H - p[0].y) / SCALE;
        ly = p[0].x / SCALE;
        if (lx < 0) lx = 0;
        if (lx >= LW) lx = LW - 1;
        if (ly < 0) ly = 0;
        if (ly >= LH) ly = LH - 1;
    }
    bool edge = pressed && !prev;
    /* One confirmation per debounced press, not per frame or drag step. */
    if (edge && s_touch_beep && (s_menu || ly >= CTRL_Y0 ||
        (ly < SPEC_Y0 && lx >= 180 && lx < 470) ||
        (s_mode == DSP_MODE_RS41 && lx >= 540 && ly >= WF_Y0 + 3 && ly < WF_Y0 + 29) ||
        ((s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) && !lora_aprs_enabled() &&
         ((lx >= FT8_BACK_X && lx < FT8_BACK_X + FT8_BACK_W &&
           ly >= FT8_BACK_Y && ly < FT8_BACK_Y + FT8_BACK_H) ||
          (lx >= FT8_FILTER_X && lx < FT8_FILTER_X + FT8_FILTER_W &&
           ly >= FT8_FILTER_Y && ly < FT8_FILTER_Y + FT8_FILTER_H))))) {
        audio_touch_beep();
    }

    /* Defer the RS41 short press until finger release so a long press cannot
     * trigger the old SNAP action first. */
    bool on_rs = pressed && hit(&B_RS41, lx, ly);
    if (edge && on_rs && !s_menu) {
        rs_hold = true;
        rs_hold_handled = false;
        rs_hold_since = pdTICKS_TO_MS(xTaskGetTickCount());
    }
    if (rs_hold && pressed && !rs_hold_handled && on_rs &&
        pdTICKS_TO_MS(xTaskGetTickCount()) - rs_hold_since >= 1500u) {
        if (s_rs_scan == RS_SCAN_OFF) rs_scan_start(); else rs_scan_stop();
        rs_hold_handled = true;
        prev = pressed;
        return true;
    }
    if (rs_hold && !pressed) {
        if (!rs_hold_handled) {
            if (s_rs_scan != RS_SCAN_OFF) rs_scan_stop();
            if (s_mode == DSP_MODE_RS41) { snap_to_peak(); s_label = "SNAP"; }
            else mode_button(DSP_MODE_RS41);
        }
        rs_hold = false;
        dragging = false;
        prev = pressed;
        return true;
    }
    if (rs_hold) { prev = pressed; return false; }

    if (s_menu) { /* an open menu owns the screen and all touches */
        if (s_menu == 12 && s_tx_mon_drag) {
            if (pressed) {
                changed = tx_mon_volume_touch(lx);
            } else {
                s_tx_mon_drag = false;
                tx_mon_volume_save();
                changed = true;
            }
            prev = pressed;
            return changed;
        }
        if (edge) changed = menu_touch(lx, ly);
        prev = pressed;
        return changed;
    }

    /* Active FT8 QSO: the entire RX spectrum + decoded-message area is a
     * return-to-QSO touch target. Handle this BEFORE pinch, message selection
     * and drag tuning so an imprecise tap cannot retune the receiver. Keep
     * the current station, AUTO sequence and TX slot untouched. */
    if (s_mode == DSP_MODE_FT8 && s_auto && s_qso_has_selected &&
        !qso_ready_for_new_station() &&
        !lora_aprs_enabled() && pressed && ly >= SPEC_Y0 && ly < CTRL_Y0) {
        if (edge) {
            s_menu = 12;
            dragging = false;
            tap_moved = false;
            multi_gesture = false;
            prev = pressed;
            return true;
        }
        dragging = false;
        prev = pressed;
        return false;
    }

    /* Two fingers on the spectrum/waterfall: zoom around the tuned centre.
     * This changes ONLY the displayed FFT window, never the SDR sample rate. */
    if (two && !lora_aprs_enabled()) {
        int x2 = (PANEL_H - p[1].y) / SCALE;
        int y2 = p[1].x / SCALE;
        if (ly < CTRL_Y0 && y2 < CTRL_Y0) {
            float dx = (float)(x2 - lx), dy = (float)(y2 - ly);
            float dist = sqrtf(dx * dx + dy * dy);
            if (!multi_gesture) {
                multi_gesture = true;
                pinch_start_dist = dist;
                pinch_start_span = view_span_hz();
            } else if (pinch_start_dist > 12 && dist > 12) {
                double target = (double)pinch_start_span * pinch_start_dist / dist;
                uint32_t full = dsp_span_hz();
                if (target < ZOOM_MIN_HZ) target = ZOOM_MIN_HZ;
                if (target > full) target = full;
                uint32_t z = (uint32_t)target;
                if (z != s_view_span) { s_view_span = z; changed = true; }
            }
            dragging = false;
            tap_moved = true;
            prev = pressed;
            return changed;
        }
    }
    if (multi_gesture) {
        /* Wait until both fingers lift; don't interpret the remaining finger as a tap. */
        if (!pressed) multi_gesture = false;
        dragging = false;
        prev = pressed;
        return changed;
    }

    /* Tap the large 7-segment frequency display to enter MHz directly. */
    if (edge && !lora_aprs_enabled() && ly < SPEC_Y0 && lx >= 180 && lx < 470) {
        snprintf(s_freq_entry, sizeof s_freq_entry, "%.4f", s_center / 1e6);
        s_menu = 9;
        dragging = false;
        prev = pressed;
        return true;
    }

    /* FT8 ALL/CQ display filter. Does not affect decoder or AUTO SEQ. */
    if (edge && (s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) && !lora_aprs_enabled() &&
        lx >= FT8_FILTER_X && lx < FT8_FILTER_X + FT8_FILTER_W &&
        ly >= FT8_FILTER_Y && ly < FT8_FILTER_Y + FT8_FILTER_H) {
        s_ft8_cq_only = !s_ft8_cq_only;
        dragging = false;
        prev = pressed;
        return true;
    }

    /* Reopen the active QSO without resetting its state or TX parity. */
    if (edge && (s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) &&
        !lora_aprs_enabled() && lx >= FT8_BACK_X && lx < FT8_BACK_X + FT8_BACK_W &&
        ly >= FT8_BACK_Y && ly < FT8_BACK_Y + FT8_BACK_H) {
        s_menu = 12;
        dragging = false;
        prev = pressed;
        return true;
    }

    /* FT8 decoded text: select any station, including an ongoing QSO. */
    if (edge && (s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) && !lora_aprs_enabled() &&
        ly >= WF_Y0 + 3 + PANEL_HDR_H && ly < WF_Y0 + 3 + PANEL_HDR_H + PANEL_ROWS * 10) {
        if (qso_select_panel_row(lx, ly)) {
            s_menu = 12;
            dragging = false;
            prev = pressed;
            return true;
        }
    }

    /* LOG is inside the waterfall area: handle its tap before drag tuning. */
    if (edge && s_mode == DSP_MODE_RS41 && !lora_aprs_enabled() &&
        lx >= 540 && lx < 632 && ly >= WF_Y0 + 3 && ly < WF_Y0 + 26) {
        dragging = false;
        tap_moved = false;
        s_log_page = 0;
        rs41_log_refresh();
        s_menu = 3;
        prev = pressed;
        return true;
    }

    /* In FT8, decoded-message rows are never a tuning gesture. Allow
     * drag/tap tuning only in the upper spectrum area (above WF_Y0).
     * This also prevents an imprecise touch from retuning the receiver. */
    if ((s_mode == DSP_MODE_FT8 || s_mode == DSP_MODE_FT4) && pressed && ly >= WF_Y0 && ly < CTRL_Y0) {
        dragging = false;
        tap_moved = true;
        prev = pressed;
        return changed;
    }
    if (pressed && ly < CTRL_Y0 && !lora_aprs_enabled() &&
        ((s_mode != DSP_MODE_FT8 && s_mode != DSP_MODE_FT4) || ly < WF_Y0)) {
        /* drag the spectrum/waterfall to scan */
        if (edge) { dragging = true; drag_x0 = lx; drag_f0 = s_center; tap_moved = false; tap_y = ly; }
        else if (dragging) {
            if (lx - drag_x0 > 6 || drag_x0 - lx > 6) tap_moved = true;
            if (tap_moved) {
                int64_t df = -(int64_t)(lx - drag_x0) * DRAG_HZ_PER_PX;
                rs_scan_stop();
                retune((int64_t)drag_f0 + df);
                s_label = "MANUAL";
                changed = true;
            }
        }
    } else if (edge) {
        dragging = false;
        if (s_mode == DSP_MODE_RS41 && lx >= 540 && ly >= WF_Y0 + 3 && ly < WF_Y0 + 29) {
            s_log_page = 0; rs41_log_refresh(); s_menu = 3; changed = true;
        } else if (lora_aprs_enabled() && !hit(&B_LORA, lx, ly) && !hit(&B_LORARX, lx, ly)) { /* LoRa owns the receiver. */ }
        else if (hit(&B_TDN, lx, ly)) { retune((int64_t)s_center - STEPS[s_step_i]); changed = true; }
        else if (hit(&B_TUP, lx, ly)) { retune((int64_t)s_center + STEPS[s_step_i]); changed = true; }
        else if (hit(&B_STEP, lx, ly)) { s_step_i = (s_step_i + 1) % N_STEPS; changed = true; }
        else if (hit(&B_GAIN, lx, ly)) {
            s_gain_i = (s_gain_i + 1) % N_GAINS;
            sdr_set_gain(GAINS[s_gain_i]);
            changed = true;
        } else if (hit(&B_RS41, lx, ly)) {
            /* RS41 is handled on release or after a 1.5s hold above. */
        } else if (ly >= MODE_BTN_Y && ly < MODE_BTN_Y + MODE_BTN_H) {
            for (int i = 0; i < N_MODE_ROW; i++) {
                box_t b = { MODE_BTN_X(i), MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H };
                if (hit(&b, lx, ly)) {
                    mode_button(MODE_ROW[i]);
                    changed = true;
                    break;
                }
            }
        } else if (hit(&B_HF, lx, ly)) {
            /* Every HF press advances to the next amateur band, even from VHF/UHF. */
            if (band_of(s_center) != BAND_HF) band_save();
            s_view_span = 0;
            rs_scan_stop();
            s_bm_hf = (s_bm_hf + 1) % BM_HF_N;
            select_bookmark(&BM_HF[s_bm_hf]);
            changed = true;
        } else if (hit(&B_VHF, lx, ly)) {
            band_press(BAND_VHF, BM_VHF, BM_VHF_N, &s_bm_vhf, "VHF LAST"); changed = true;
        } else if (hit(&B_UHF, lx, ly)) {
            band_press(BAND_UHF, BM_UHF, BM_UHF_N, &s_bm_uhf, "UHF LAST"); changed = true;
        } else if (hit(&B_PSK, lx, ly)) {
            psk_set_enabled(!psk_enabled()); changed = true;
        } else if (hit(&B_LORARX, lx, ly)) {
            /* Same LoRa RX workflow formerly launched from the PLAN menu. */
            lora_rx_get_config(&s_lr_edit);
            s_lr_saved = false;
            lora_rx_enable(true);
            if (lora_rx_enabled()) {
                sdr_set_iq_discard(true);
                audio_set_volume(0);
                s_menu = 7;
            } else {
                s_lora_error_until = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
            }
            changed = true;
        } else if (hit(&B_HORUS, lx, ly)) {
            mode_button(DSP_MODE_HORUS);
            changed = true;
        } else if (hit(&B_APRS, lx, ly)) {
            s_view_span = 0; /* Reset also when toggling APRS VHF/UHF. */
            if (s_mode == DSP_MODE_APRS) {
                uint32_t f = s_center == 144800000u ? 432500000u : 144800000u;
                retune(f);
                aprs_reset();
                s_label = f == 144800000u ? "APRS 1200 VHF" : "APRS 1200 UHF";
            } else mode_button(DSP_MODE_APRS);
            s_panel_mode = -1;
            s_ui_force_spectrum = true;
            changed = true;
        } else if (hit(&B_LORA, lx, ly)) {
            s_view_span = 0; /* Reset on LoRa APRS entry/exit. */
            if (!lora_aprs_enabled() || lora_rx_enabled()) {
                /* Keep the USB host/async task alive. The IQ callback discards
                 * samples during LoRa mode; DSP stops processing them. */
                lora_aprs_enable(true);
                if (lora_aprs_enabled()) {
                    sdr_set_iq_discard(true);
                    audio_set_volume(0);
                    ESP_LOGI(TAG, "L-APRS enabled; RTL IQ discarded");
                } else {
                    /* Radio failed to enable: never leave the speaker muted. */
                    sdr_set_iq_discard(false);
                    audio_set_volume(s_volume);
                    ESP_LOGE(TAG, "L-APRS enable FAILED (check RA-02/SPI init)");
                    s_lora_error_until = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
                }
            } else {
                /* Resume IQ delivery without cancelling/restarting USB async. */
                sdr_set_iq_discard(false);
                lora_aprs_enable(false);
                audio_set_volume(s_volume);
                s_panel_mode = -1;
                rect(0, SPEC_Y0, LW, WF_Y0 + WF_H - SPEC_Y0, COL_BG);
            }
            s_ui_force_spectrum = true;
            changed = true;
        } else if (hit(&B_PLAN, lx, ly)) {
            s_menu = 1; changed = true;
        } else if (hit(&B_VDN, lx, ly)) {
            s_volume -= VOL_STEP; if (s_volume < 0) s_volume = 0;
            audio_set_volume(s_volume); changed = true;
        } else if (hit(&B_VUP, lx, ly)) {
            s_volume += VOL_STEP; if (s_volume > 100) s_volume = 100;
            audio_set_volume(s_volume); changed = true;
        } else if (hit(&B_SDN, lx, ly)) {
            s_squelch -= SQ_STEP; if (s_squelch < -60) s_squelch = -60;
            apply_squelch(); changed = true;
        } else if (hit(&B_SUP, lx, ly)) {
            s_squelch += SQ_STEP; if (s_squelch > 60) s_squelch = 60;
            apply_squelch(); changed = true;
        }
    }

    if (!pressed) {
        /* Preserve WSPR panel self-test; otherwise a tap tunes to that FFT position. */
        if (dragging && !tap_moved) {
            if (s_mode == DSP_MODE_WSPR && tap_y >= WF_Y0) {
                wspr_selftest(); changed = true;
            } else if (s_mode != DSP_MODE_APRS &&
                       (tap_y < WF_Y0 || !is_panel_mode(s_mode))) {
                int64_t offset = ((int64_t)drag_x0 - LW / 2) * view_span_hz() / LW;
                retune((int64_t)drag_f0 + offset);
                s_label = "TAP TUNE";
                changed = true;
            }
        }
        dragging = false;
    }
    prev = pressed;
    return changed;
}

/* ---- present ------------------------------------------------------------- */

static void present(void)
{
    /* Keep the web image alive while the LCD backlight is off.
     * Do not run the LCD PPA/flush while the screensaver is active. */
    if (s_scr_off) {
        if (web_mirror_enabled()) web_mirror_publish(LBUF);
        return;
    }
    ppa_do_scale_rotate_mirror(s_ppa, &(ppa_srm_oper_config_t){
        .in = { .buffer = LBUF, .pic_w = LW, .pic_h = LH, .block_w = LW, .block_h = LH,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .out = { .buffer = bsp_tab5_display_get_frame_buffer(s_fb_index),
                 .buffer_size = PANEL_W * PANEL_H * 2, .pic_w = PANEL_W, .pic_h = PANEL_H,
                 .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_90, .scale_x = SCALE, .scale_y = SCALE,
    });
    bsp_tab5_display_flush(s_fb_index);
    web_mirror_publish(LBUF);
    s_fb_index ^= 1;
}

static void ui_task(void *arg)
{
    (void)arg;
    rect(0, 0, LW, LH, COL_BG);
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0));

    /* Animated startup splash: 2.5 seconds, no external fonts/assets. */
    const char *welcome_call = station_call();
    const int title_sc = 7, call_sc = 5;
    const int title_x = (LW - text_w(SPLASH_TITLE, title_sc)) / 2;
    const int call_x = (LW - text_w(welcome_call, call_sc)) / 2;
    for (int frame = 0; frame < 25; ++frame) {
        /* Dark navy background with subtle horizontal highlights. */
        rect(0, 0, LW, LH, RGB565(5, 10, 25));
        for (int y = 0; y < LH; y += 8) {
            int glow = 10 + (LH / 2 - (y > LH / 2 ? y - LH / 2 : LH / 2 - y)) / 24;
            rect(0, y, LW, 1, RGB565(5, glow, 30 + glow));
        }
        /* Decorative corner brackets and radio-style horizontal rules. */
        const uint16_t edge = RGB565(23, 95, 124);
        rect(38, 38, 92, 2, edge); rect(38, 38, 2, 36, edge);
        rect(LW - 130, 38, 92, 2, edge); rect(LW - 40, 38, 2, 36, edge);
        rect(38, LH - 40, 92, 2, edge); rect(38, LH - 74, 2, 36, edge);
        rect(LW - 130, LH - 40, 92, 2, edge); rect(LW - 40, LH - 74, 2, 36, edge);

        /* A small moving RF sweep above the title. */
        const int sweep_w = (LW - 180) * (frame + 1) / 25;
        rect(90, 98, LW - 180, 2, RGB565(25, 44, 67));
        rect(90, 98, sweep_w, 2, RGB565(0, 180, 185));
        if (sweep_w > 4) rect(90 + sweep_w - 4, 94, 4, 10, RGB565(130, 245, 235));

        /* Reveal title character-by-character with a restrained shadow. */
        char title_reveal[sizeof(SPLASH_TITLE)] = {0};
        int visible = frame < 4 ? frame + 1 : (int)sizeof(SPLASH_TITLE) - 1;
        if (visible > (int)sizeof(SPLASH_TITLE) - 1) visible = (int)sizeof(SPLASH_TITLE) - 1;
        memcpy(title_reveal, SPLASH_TITLE, (size_t)visible);
        text(title_x + 3, 141 + 3, title_sc, RGB565(9, 47, 61), title_reveal);
        text(title_x, 141, title_sc, RGB565(85, 238, 219), title_reveal);

        /* Station identity fades in after the title. */
        if (frame >= 6) {
            const int level = (frame - 5) * 24 > 255 ? 255 : (frame - 5) * 24;
            text(call_x + 2, 226 + 2, call_sc, RGB565(40, 35, 13), welcome_call);
            text(call_x, 226, call_sc,
                 RGB565((255 * level) / 255, (195 * level) / 255, (72 * level) / 255),
                 welcome_call);
        }
        /* Animated progress indicator, independent of SDR/USB startup. */
        rect(150, LH - 95, LW - 300, 5, RGB565(27, 43, 61));
        rect(150, LH - 95, (LW - 300) * (frame + 1) / 25, 5,
             RGB565(0, 187, 152));
        present();
        vTaskDelay(pdMS_TO_TICKS(SPLASH_MS / 25));
    }

    cpu_temp_init();
    cpu_temp_sample();
    draw_controls();
    draw_header();
    present();

    const float *last = NULL;
    int bins = dsp_spectrum_bins();
    int meter_div = 0;
    int last_psk = -1;
    TickType_t last_temp_tick = xTaskGetTickCount();
    uint32_t last_lora_revision = 0;
    uint32_t last_aprs_revision = 0;
    scr_off_kick(); /* the splash counts as activity */
    int previous_menu = s_menu;
    while (1) {
        bool dirty = handle_touch();
        /* A full-screen menu leaves pixels behind when returning to the main
         * SDR view. Invalidate the cached spectrum/panel and rebuild the
         * entire framebuffer, without touching the DSP or FT8 decoder. */
        if (previous_menu != 0 && s_menu == 0) {
            rect(0, 0, LW, LH, COL_BG);
            rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0));
            s_panel_mode = -1;
            s_panel_total = 0;
            last = NULL;
            s_ui_force_spectrum = true;
            dirty = true;
        }
        previous_menu = s_menu;
        /* Process directed CQ replies before scheduling AUTO TX. */
        if (cq_tick()) dirty = true;
        if (ft4_cq_tick()) dirty = true;
        if (qso_auto_tick()) dirty = true;
        if (ft4_auto_tick()) dirty = true;
        if (qso_history_rx_tick()) dirty = true;
        if (rs_scan_tick()) dirty = true;
        /* Do not let an active FT8/WSPR schedule override manual HORUS reception. */
        if (!lora_aprs_enabled() && s_mode != DSP_MODE_HORUS && s_mode != DSP_MODE_APRS && s_rs_scan == RS_SCAN_OFF && bandplan_tick()) dirty = true;
        /* screensaver: idle timeout turns the backlight off; a touch wakes it */
        if (!s_scr_off && SCR_OFF_MS[s_scr_off_i] &&
            pdTICKS_TO_MS(xTaskGetTickCount()) - s_last_touch_ms >= SCR_OFF_MS[s_scr_off_i])
            scr_off_set(true);
        if (s_scr_off && !web_mirror_enabled()) { vTaskDelay(pdMS_TO_TICKS(50)); continue; } /* LCD off; render only if web mirror needs frames */
        if (s_menu) { /* an open menu owns the screen; the SDR keeps running underneath */
            /* A menu may render only once. The web publisher is rate-limited to
             * 100 ms, so that one render can be skipped. Retry publishing the
             * same stable LBUF for a short time after a menu change. This does
             * not redraw the LCD or alter the SDR, and guarantees that the
             * mirror eventually captures the menu even if the first call was
             * throttled or its frame mutex was busy. */
            static int mirror_menu_retry = 0;
            if ((s_menu == 11 || s_menu == 12) && si5351_tone_test_running()) dirty = true;
            if (s_menu == 12 && s_auto) dirty = true;
            if (s_menu == 12) {
                static bool last_qso_rf_tx = false;
                bool rf_tx_now = si5351_relay_is_tx() && si5351_tone_test_index() >= 0;
                if (rf_tx_now != last_qso_rf_tx) {
                    last_qso_rf_tx = rf_tx_now;
                    dirty = true;
                }
            }
            if (s_menu == 7 && lora_aprs_revision()!=last_lora_revision) {last_lora_revision=lora_aprs_revision();dirty=true;}
            if (dirty) {
                if (s_menu == 3 || s_menu == 4) rs41_log_draw();
                else draw_menu();
                present();
                mirror_menu_retry = 8;
            }
            if (mirror_menu_retry > 0) {
                web_mirror_publish(LBUF);
                mirror_menu_retry--;
            }
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        TickType_t now_temp = xTaskGetTickCount();
        if (now_temp - last_temp_tick >= pdMS_TO_TICKS(5000)) {
            last_temp_tick = now_temp;
            cpu_temp_sample();
            dirty = true;
        }
        int ps = psk_status(); /* PSK box follows WiFi/SNTP state by itself */
        if (ps != last_psk) { last_psk = ps; dirty = true; }
        uint32_t lr = lora_aprs_revision();
        if (lr != last_lora_revision) { last_lora_revision = lr; dirty = true; }
        uint32_t ar = aprs_revision();
        if (ar != last_aprs_revision) { last_aprs_revision = ar; dirty = true; }
        if (s_ui_force_spectrum) { last = NULL; s_ui_force_spectrum = false; }
        const float *spec = dsp_spectrum();
        bool newspec = spec && spec != last;
        if (lora_aprs_enabled() && dirty) {
            draw_lora_panel();
            dirty = true;
        }
        if (newspec && !lora_aprs_enabled()) {
            last = spec;
            draw_spectrum(spec, bins);
            if (++meter_div >= 4) { meter_div = 0; draw_header(); }
        }
        if (dirty) {
            if (s_mode == DSP_MODE_APRS && !lora_aprs_enabled()) draw_aprs_panel();
            draw_controls(); if (!lora_aprs_enabled()) draw_header(); else {
            rect(0, 0, LW, SPEC_Y0 - 1, COL_BG);
            text(8, 8, 2, COL_ACCENT, "LORA APRS");
            text(350, 14, 1, COL_LINE, "RTL-SDR PAUSED");
            draw_cpu_temp();
        } }
        if (s_lora_error_until &&
            (int32_t)(s_lora_error_until - xTaskGetTickCount()) > 0) {
            /* Overlay on the spectrum; keep it visible even when no new IQ arrives. */
            rect(0, SPEC_Y0, LW, 32, RGB565(90, 12, 12));
            text(8, SPEC_Y0 + 6, 2, COL_TEXT, "LORA ERROR - CHECK RA-02 / SPI");
            dirty = true;
        } else if (s_lora_error_until) {
            s_lora_error_until = 0;
            s_ui_force_spectrum = true;
            dirty = true;
        }
        if (newspec || dirty) present();
        vTaskDelay(pdMS_TO_TICKS(is_panel_mode(s_mode) ? 30 : 8)); /* text panels: ~30 fps is plenty, and the decoders need CPU */
    }
}

void ui_start(void)
{
    LBUF = heap_caps_malloc((size_t)LW * LH * 2, MALLOC_CAP_SPIRAM);
    if (!LBUF) { ESP_LOGE(TAG, "no PSRAM for landscape buffer"); return; }
    ESP_ERROR_CHECK(ppa_register_client(&(ppa_client_config_t){ .oper_type = PPA_OPERATION_SRM }, &s_ppa));

    sdr_stats_t st;
    sdr_get_stats(&st);
    s_center = st.center_freq;
    audio_set_volume(s_volume);
    apply_squelch();

    /* ui_start runs before wifi_net_init (which also inits NVS): make sure the
     * flash is ready so the screensaver setting and the band plan load here. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() == ESP_OK) nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK && nvs_err != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "nvs_flash_init: %s", esp_err_to_name(nvs_err));

    scr_off_load();
    touch_beep_load();
    tx_mon_volume_load();
    cq_tx_audio_load();
    bandplan_init();
    bandplan_set_apply_cb(plan_apply);

    ESP_LOGI(TAG, "landscape UI %dx%d (PPA x%d)", LW, LH, SCALE);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 6, NULL, 0);
}
