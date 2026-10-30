/*
 * ui.c — landscape touch UI for Tab5-SDR.
 *
 * Rendered into a 640x360 landscape RGB565 buffer (LBUF); the P4 PPA scales x2
 * and rotates 90° onto the 720x1280 panel (half-res keeps the PPA cheap).
 * Touch maps back: lx = (1280 - panel_y)/2,  ly = panel_x/2.
 *
 * Controls: drag spectrum/waterfall to scan fast; [-1][-.1][+.1][+1] tune;
 * mode row WFM/NFM/AM/USB/LSB/CW/FT8/WSPR, RS41 button (press again = centre on peak);
 * HF/VHF/UHF band buttons; PLAN button: time-scheduled band hopping for
 * FT8/WSPR (menu: OFF / FT8 PLAN / WSPR PLAN / USER PLAN / editor / SCREEN OFF);
 * volume +/- ; squelch +/-. The SCREEN OFF row sets the display-off (screensaver)
 * delay; after the idle timeout the backlight turns off and any touch wakes it.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <strings.h>
#include <sys/stat.h>
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
#include "ft8.h"
#include "wspr.h"
#include "audio.h"
#include "bookmarks.h"
#include "psk.h"
#include "bandplan.h"
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
#define DRAG_HZ_PER_PX (10000000 / LW)
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
#define SPLASH_CALL  "9A3ZI"
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
static int s_bm_hf = 0, s_bm_vhf = 0, s_bm_uhf = 0; /* per-band preset cursor */
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
    rect(0, 0, LW, SPEC_Y0 - 1, COL_BG);

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
    text(555, WF_Y0 + 8, 2, COL_TEXT, "LOG");
    const int x = 8, pitch = 18;
    int y = WF_Y0 + 4;
    char b[128];
    if (!have) {
        text(x, y, 2, RGB565(230, 230, 230), "WAITING FOR RS41");
        text(x, y + pitch, 2, RGB565(150, 150, 150), "TUNE 400-406 MHZ");
        text(x, y + 2 * pitch, 2, RGB565(150, 150, 150), "PRESS RS41 AGAIN TO CENTRE ON THE PEAK");
        return;
    }
    if (in.seq != last_seq) { last_seq = in.seq; last_tick = xTaskGetTickCount(); }
    bool stale = (xTaskGetTickCount() - last_tick) > pdMS_TO_TICKS(4000);
    uint16_t cv = stale ? RGB565(120, 120, 120) : RGB565(230, 230, 230);
    uint16_t ci = stale ? RGB565(120, 120, 120) : RGB565(255, 220, 0);

    snprintf(b, sizeof b, "ID %s  FRM %u", in.have_id ? in.id : "-", (unsigned)in.frame_no);
    text(x, y, 2, ci, b);
    y += pitch;
    if (in.have_pos) snprintf(b, sizeof b, "LAT %.5f  LON %.5f", in.lat, in.lon);
    else if (in.encrypted) snprintf(b, sizeof b, "POSITION ENCRYPTED  RS41 SGM");
    else snprintf(b, sizeof b, "LAT -  LON -");
    text(x, y, 2, cv, b);
    y += pitch;
    if (in.have_pos) snprintf(b, sizeof b, "ALT %.0f M  VH %.1f  VV %.1f  HDG %.0f", in.alt, in.vh, in.vv, in.vd);
    else snprintf(b, sizeof b, "ALT -");
    text(x, y, 2, cv, b);
    y += pitch;
    if (in.have_time) snprintf(b, sizeof b, "UTC %02d:%02d:%02d  SAT %u", in.hh, in.mm, in.ss, (unsigned)in.sats);
    else snprintf(b, sizeof b, "UTC -");
    text(x, y, 2, cv, b);
    y += pitch;
    if (in.rs_fixed < 0) snprintf(b, sizeof b, "OK %u  BAD %u  RS FAIL  MK %u/7  BLK %u", (unsigned)in.frames_ok, (unsigned)in.frames_bad, (unsigned)in.mk, (unsigned)in.blk_ok);
    else snprintf(b, sizeof b, "OK %u  BAD %u  RS %d  MK %u/7  BLK %u", (unsigned)in.frames_ok, (unsigned)in.frames_bad, in.rs_fixed, (unsigned)in.mk, (unsigned)in.blk_ok);
    text(x, y, 2, RGB565(150, 150, 150), b);
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

static inline bool is_panel_mode(int m) { return m == DSP_MODE_RS41 || m == DSP_MODE_FT8 || m == DSP_MODE_WSPR; }

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

static void draw_ft8_panel(void)
{
    ft8_msg_t msgs[PANEL_COLS * PANEL_ROWS];
    ft8_status_t st;
    int n = ft8_get_messages(msgs, PANEL_COLS * PANEL_ROWS, &st); /* oldest first */
    bool body = panel_begin(DSP_MODE_FT8, st.total);

    char b[100];
    if (st.locked)
        snprintf(b, sizeof b, "FT8  SLOT SYNC OK   T %04.1f S   SLOTS %u   MSGS %u", (double)st.slot_pos,
                 (unsigned)st.slots, (unsigned)st.total);
    else
        snprintf(b, sizeof b, "FT8  SCANNING FOR THE SLOT TIMING   MSGS %u", (unsigned)st.total);
    text(6, WF_Y0 + 3, 1, st.locked ? RGB565(60, 220, 140) : RGB565(255, 190, 60), b);
    if (!body) return;

    if (n == 0) {
        text(6, WF_Y0 + 3 + PANEL_HDR_H, 1, RGB565(170, 170, 170), "NO MESSAGES YET. TUNE TO AN FT8 FREQUENCY IN USB,");
        text(6, WF_Y0 + 3 + PANEL_HDR_H + 10, 1, RGB565(170, 170, 170), "E.G. 14.074 MHZ (20 M). THE FIRST DECODE SETS THE SLOT TIMING.");
        return;
    }
    panel_separators();
    uint16_t newest_slot = msgs[n - 1].slot;
    for (int k = 0; k < n; k++) {
        const ft8_msg_t *m = &msgs[n - 1 - k]; /* k = 0 is the newest */
        snprintf(b, sizeof b, "%4d %+3d %.*s", (int)m->freq, (int)m->snr, PANEL_TEXT_CH, m->text);
        uint16_t color = (m->slot == newest_slot) ? RGB565(235, 235, 235) : RGB565(120, 120, 120);
        if (m->slot == newest_slot && m->text[0] == 'C' && m->text[1] == 'Q') color = RGB565(255, 220, 0); /* CQ calls */
        panel_line(k, n, color, b);
    }
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
    text(8, SPEC_Y0 + 4, 2, COL_ACCENT, b);
    snprintf(b, sizeof b, "OK %lu  CRC ERR %lu  FLAGS %lu  AFSK %d%%", (unsigned long)aprs_packet_count(),
             (unsigned long)aprs_crc_errors(), (unsigned long)aprs_flags(), aprs_quality_pct());
    text(8, SPEC_Y0 + 26, 1, COL_LINE, b);
    aprs_packet_t pk[APRS_HISTORY];
    int n = aprs_packets(pk, APRS_HISTORY);
    if (!n) { text(8, SPEC_Y0 + 55, 2, COL_TEXT, "WAITING FOR APRS PACKET..."); return; }
    for (int i = 0; i < n; ++i) {
        char line[101];
        snprintf(line, sizeof line, "%.100s", pk[i].text);
        line[100] = 0;
        text(8, SPEC_Y0 + 46 + i * 22, 1, i ? COL_TEXT : COL_ACCENT, line);
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
        b[68] = 0;
        horus_text(8, SPEC_Y0 + 47, COL_TEXT, b);
        horus_text(8, SPEC_Y0 + 68, COL_ACCENT, "WAITING FOR APRS PACKET...");
        return;
    }
    for (int i = 0; i < n; ++i) {
        int y = SPEC_Y0 + 43 + i * 25;
        snprintf(b, sizeof b, "%d RSSI:%d SNR:%+.1f", i + 1, p[i].rssi_dbm, (double)p[i].snr_db);
        text(8, y, 1, i == 0 ? COL_LINE : COL_ACCENT, b);
        snprintf(b, sizeof b, "%s", p[i].text);
        b[68] = 0;
        horus_text(8, y + 9, COL_TEXT, b);
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
    else if (s_mode == DSP_MODE_WSPR) draw_wspr_panel();
    if (lora_aprs_enabled() || s_mode == DSP_MODE_HORUS) {
        char msg[192];
        if (s_mode == DSP_MODE_HORUS) {
            /* HORUS: six medium-font lines (1.5x) with more vertical spacing.
             * Keep FT8/WSPR, other UI panels and decoder logic untouched. */
            const int hx = 8, hy = WF_Y0 + 3, hp = 16;
            rect(4, WF_Y0 + 2, 632, WF_H - 4, COL_BG);
            horus_decoder_status(msg, sizeof msg);
            msg[68] = 0; /* 68 glyphs * 9 px = 612 px */
            horus_text(hx, hy, COL_ACCENT, msg);
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
                msg[68] = 0;
                horus_text(hx, hy + hp, COL_LINE, msg);
                snprintf(msg, sizeof msg, "UTC %02u:%02u:%02u", v.hour, v.minute, v.second);
                horus_text(hx, hy + 2 * hp, COL_TEXT, msg);
                snprintf(msg, sizeof msg, "LAT %.6f  LON %.6f",
                         (double)v.latitude, (double)v.longitude);
                msg[68] = 0;
                horus_text(hx, hy + 3 * hp, COL_TEXT, msg);
                snprintf(msg, sizeof msg, "ALT %u m  SPD %u km/h  SAT %u",
                         v.altitude, v.speed, v.satellites);
                msg[68] = 0;
                horus_text(hx, hy + 4 * hp, COL_TEXT, msg);
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
                horus_text(hx, hy + 5 * hp, COL_TEXT, msg);
            } else {
                char raw[260]; unsigned ver = 0, len = 0; bool parsed = false;
                horus_decoder_raw(raw, sizeof(raw), &ver, &len, &parsed);
                if (len) {
                    snprintf(msg, sizeof msg, "CRC OK  V%u  HEX:%u  PARSE:%s",
                             ver, len, parsed ? "OK" : "NE");
                    horus_text(hx, hy + hp, COL_TEXT, msg);
                    raw[68] = 0;
                    horus_text(hx, hy + 2 * hp, COL_TEXT, raw);
                } else {
                    horus_text(hx, hy + 2 * hp, COL_TEXT, "CEKAM HORUS PAKET...");
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
static const box_t B_RS41 = {210, 220, 88, 44};  /* RS41 mode; pressed again in RS41 mode = centre on the strongest peak */
static const box_t B_GAIN = {306, 220, 44, 44};  /* tuner gain: AGC / manual steps */
static const box_t B_PLAN = {358, 220, 88, 44};  /* band-plan menu: time-scheduled FT8/WSPR band hopping */
static const box_t B_HF   = {  6, 272, 64, 44};
static const box_t B_VHF  = { 74, 272, 64, 44};
static const box_t B_UHF  = {142, 272, 64, 44};
static const box_t B_PSK  = {210, 272, 72, 44}; /* PSK Reporter upload checkbox */
static const box_t B_HORUS = {286, 272, 50, 44};
static const box_t B_LORA = {339, 272, 54, 44};
static const box_t B_APRS = {396, 272, 52, 44};
/* Mode row: WFM NFM AM USB LSB CW FT8 WSPR — 8 buttons, 52 px wide, 3 px gap (RS41 has its own
 * button). The row ends at x = 443; VOL/SQL start at x = 454. */
static const uint8_t MODE_ROW[] = {DSP_MODE_WFM, DSP_MODE_NFM, DSP_MODE_AM, DSP_MODE_USB, DSP_MODE_LSB, DSP_MODE_CW, DSP_MODE_FT8, DSP_MODE_WSPR};
#define N_MODE_ROW (int)(sizeof(MODE_ROW) / sizeof(MODE_ROW[0]))
#define MODE_BTN_W 52
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
                char st[24];
                snprintf(st, sizeof st, "%s %s", dsp_mode_name(cur.mode),
                         bandplan_band_name(bandplan_slot_freq(&cur), bn, sizeof bn));
                text(B_PLAN.x + (B_PLAN.w - text_w(st, 1)) / 2, B_PLAN.y + 28, 1, RGB565(190, 255, 210), st);
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

static void retune(int64_t hz)
{
    if (hz < FREQ_MIN) hz = FREQ_MIN;
    if (hz > FREQ_MAX) hz = FREQ_MAX;
    int64_t old = s_center;
    int rc = sdr_set_center_freq((uint32_t)hz);
    if (rc < 0) {
        ESP_LOGE(TAG, "RTL-SDR retune FAILED: requested %lld Hz (rc=%d)", (long long)hz, rc);
        return;
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

/* Digital-mode button: narrow 96 kHz view, 100 Hz step; pressed again = next frequency of the
 * list, otherwise keep the dial if it is already on one of the frequencies, else jump to the
 * last used one. */
static void digital_preset(const uint32_t *list, int n, int *idx, bool again)
{
    set_rate(288000);
    set_step_hz(100);
    if (again) {
        *idx = (*idx + 1) % n;
        retune(list[*idx]);
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
static int s_menu;
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
            char out[96];
            /* Show first fields: serial, frame, frequency and GPS week. */
            snprintf(out, sizeof out, "%.72s", line);
            out[strcspn(out, "\r\n")] = 0;
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
#define M1_ROWS      6
static const int s_plan_menu_actions[M1_ROWS] = {BP_OFF, BP_USER, -1, -2, -3, -4};
#define M1_ROW_Y0    48
#define M1_ROW_PITCH 32
#define M1_ROW_H     28
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
        text(12,8,2,COL_ACCENT,"STATION SETTINGS");
        text(12,32,1,COL_TEXT,"TAP FIELD TO EDIT - SAVE WRITES FLASH NVS");
        const char *labels[4] = {"WIFI SSID","WIFI PASSWORD","CALLSIGN","QTH LOCATOR"};
        const char *values[4] = {s_st_edit.ssid,"********",s_st_edit.callsign,s_st_edit.grid};
        for (int i=0;i<4;i++) {
            int y=58+i*48;
            rect(12,y,616,42,COL_BTN);
            text(22,y+6,2,COL_TEXT,labels[i]);
            text(290,y+6,2,RGB565(130,230,255),values[i]);
        }
        rect(12,264,294,44,RGB565(25,100,65));
        text(95,276,2,COL_TEXT,"SAVE");
        rect(334,264,294,44,COL_BTN);
        text(424,276,2,COL_TEXT,"BACK");
        if (s_st_saved) text(14,314,1,RGB565(150,255,160),"SAVED - WIFI RECONNECTING");
        return;
    }
    const char *labels[4]={"WIFI SSID","WIFI PASSWORD","CALLSIGN","QTH LOCATOR"};
    text(12,6,2,COL_ACCENT,labels[s_st_field]);
    char shown[72];
    const char *v=settings_field();
    if (s_st_field == 1) {
        size_t n=strlen(v); if(n>60)n=60;
        memset(shown,'*',n); shown[n]=0;
    } else snprintf(shown,sizeof shown,"%s",v);
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
        if(y>=58 && y<250) {int i=(y-58)/48; if(i<4){s_st_field=i;s_st_lower=(i<=1);s_menu=6;}return true;}
        if(y>=264 && y<310) {
            if(x<318) {
                s_st_saved=station_settings_save(&s_st_edit);
                if(s_st_saved)wifi_net_apply_settings();
            } else {s_menu=1;}
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
                 action == -3 ? "SETTINGS" : "CLOSE");
            text(r.x + 12, r.y + 8, 2, COL_TEXT, label);
            if (action >= 0) {
                plan_info(action, b, sizeof b);
                text(r.x + 300, r.y + 11, 1, RGB565(170, 180, 200), b);
            } else if (action == -2) {
                text(r.x + 300, r.y + 11, 1, s_scr_off_i ? RGB565(190, 255, 210) : RGB565(170, 180, 200),
                     SCR_OFF_LBL[s_scr_off_i]);
            }
        }
        return;
    }
    if (s_menu == 5 || s_menu == 6) { settings_draw(); return; }
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
    if (s_menu == 5 || s_menu == 6) return settings_touch(x,y);
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

    /* ~1.2 s startup grace: the touch controller reports stray points right after
     * init that would otherwise "tap" a control. */
    if (grace < 150) { grace++; prev = false; run = 0; return false; }

    /* Debounce: a press must persist for 2 polls (~16 ms) before it counts,
     * rejecting single-sample glitches. */
    run = (n > 0) ? run + 1 : 0;
    bool pressed = run >= 2;
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
        if (edge) changed = menu_touch(lx, ly);
        prev = pressed;
        return changed;
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

    if (pressed && ly < CTRL_Y0 && !lora_aprs_enabled()) {
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
        } else if (lora_aprs_enabled() && !hit(&B_LORA, lx, ly)) { /* LoRa owns the receiver. */ }
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
            band_press(BAND_HF, BM_HF, BM_HF_N, &s_bm_hf, "HF LAST"); changed = true;
        } else if (hit(&B_VHF, lx, ly)) {
            band_press(BAND_VHF, BM_VHF, BM_VHF_N, &s_bm_vhf, "VHF LAST"); changed = true;
        } else if (hit(&B_UHF, lx, ly)) {
            band_press(BAND_UHF, BM_UHF, BM_UHF_N, &s_bm_uhf, "UHF LAST"); changed = true;
        } else if (hit(&B_PSK, lx, ly)) {
            psk_set_enabled(!psk_enabled()); changed = true;
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
            if (!lora_aprs_enabled()) {
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
    ppa_do_scale_rotate_mirror(s_ppa, &(ppa_srm_oper_config_t){
        .in = { .buffer = LBUF, .pic_w = LW, .pic_h = LH, .block_w = LW, .block_h = LH,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .out = { .buffer = bsp_tab5_display_get_frame_buffer(s_fb_index),
                 .buffer_size = PANEL_W * PANEL_H * 2, .pic_w = PANEL_W, .pic_h = PANEL_H,
                 .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_90, .scale_x = SCALE, .scale_y = SCALE,
    });
    bsp_tab5_display_flush(s_fb_index);
    s_fb_index ^= 1;
}

static void ui_task(void *arg)
{
    (void)arg;
    rect(0, 0, LW, LH, COL_BG);
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0));

    /* welcome splash: title + callsign centred on the whole display */
    rect(0, 0, LW, LH, COL_PANEL);
    text((LW - text_w(SPLASH_TITLE, 6)) / 2, 126, 6, COL_ACCENT, SPLASH_TITLE);
    text((LW - text_w(SPLASH_CALL, 4)) / 2, 180, 4, COL_LINE, SPLASH_CALL);
    present();
    vTaskDelay(pdMS_TO_TICKS(SPLASH_MS));

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
    while (1) {
        bool dirty = handle_touch();
        if (rs_scan_tick()) dirty = true;
        /* Do not let an active FT8/WSPR schedule override manual HORUS reception. */
        if (!lora_aprs_enabled() && s_mode != DSP_MODE_HORUS && s_mode != DSP_MODE_APRS && s_rs_scan == RS_SCAN_OFF && bandplan_tick()) dirty = true;
        /* screensaver: idle timeout turns the backlight off; a touch wakes it */
        if (!s_scr_off && SCR_OFF_MS[s_scr_off_i] &&
            pdTICKS_TO_MS(xTaskGetTickCount()) - s_last_touch_ms >= SCR_OFF_MS[s_scr_off_i])
            scr_off_set(true);
        if (s_scr_off) { vTaskDelay(pdMS_TO_TICKS(50)); continue; } /* dark: nothing to render */
        if (s_menu) { /* an open menu owns the screen; the SDR keeps running underneath */
            if (dirty) { if (s_menu == 3 || s_menu == 4) rs41_log_draw(); else draw_menu(); present(); }
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
    bandplan_init();
    bandplan_set_apply_cb(plan_apply);

    ESP_LOGI(TAG, "landscape UI %dx%d (PPA x%d)", LW, LH, SCALE);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 6, NULL, 0);
}
