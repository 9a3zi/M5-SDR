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
 * FT8/WSPR (menu: OFF / FT8 PLAN / WSPR PLAN / USER PLAN / editor); volume +/- ;
 * squelch +/-.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/ppa.h"

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

static const char *TAG = "ui";

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
#define DRAG_HZ_PER_PX (100000000 / LW)

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
#define SPLASH_TITLE "TAB-5 SDR"
#define SPLASH_CALL  "9A3ZI"
#define SPLASH_MS    2500

static uint16_t *LBUF;
static ppa_client_handle_t s_ppa;
static int s_fb_index;

static uint32_t s_center;
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
    default:           return COL_ACCENT; /* WFM */
    }
}

/* ---- header: freq + mode/label + S-meter --------------------------------- */

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
    text(6, 24, 1, COL_TEXT, s_label);

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

static void draw_spectrum(const float *spec, int bins)
{
    const bool rs = is_panel_mode(s_mode); /* mode with a text panel instead of the waterfall */
    if (!rs)
        memmove(LBUF + (size_t)(WF_Y0 + 1) * LW, LBUF + (size_t)WF_Y0 * LW,
                (size_t)(WF_H - 1) * LW * sizeof(uint16_t));
    int cx = LW / 2;
    for (int x = 0; x < LW; x++) {
        int bin = (x * bins) / LW;
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
}

/* ---- control band -------------------------------------------------------- */

typedef struct { int x, y, w, h; } box_t;
static const box_t B_TDN  = {  6, 220, 52, 44};  /* tune down one step */
static const box_t B_STEP = { 62, 220, 84, 44};  /* cycle step size    */
static const box_t B_TUP  = {150, 220, 52, 44};  /* tune up one step   */
static const box_t B_RS41 = {210, 220, 88, 44};  /* RS41 mode; pressed again in RS41 mode = centre on the strongest peak */
static const box_t B_GAIN = {306, 220, 44, 44};  /* tuner gain: AGC / manual steps */
static const box_t B_PLAN = {358, 220, 88, 44};  /* band-plan menu: time-scheduled FT8/WSPR band hopping */
static const box_t B_HF   = {  6, 272, 108, 44};
static const box_t B_VHF  = {118, 272, 108, 44};
static const box_t B_UHF  = {230, 272, 108, 44};
static const box_t B_PSK  = {344, 272, 104, 44}; /* PSK Reporter upload checkbox */
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
    { /* PSK Reporter upload checkbox: green = enabled + online, amber = enabled, no link yet */
        int st = psk_status();
        draw_box(&B_PSK, "PSK", 2, COL_BTN);
        rect(B_PSK.x + 10, B_PSK.y + 13, 18, 18, COL_TEXT);
        if (st) rect(B_PSK.x + 13, B_PSK.y + 16, 12, 12, st == 2 ? RGB565(40, 190, 90) : RGB565(220, 170, 40));
    }

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
    s_center = (uint32_t)hz;
    sdr_set_center_freq(s_center);
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
    const bool again = (s_mode == m); /* same mode pressed twice */
    set_mode(m);
    s_label = "MANUAL";
    /* sensible default step: 100 Hz for SSB/CW (SSB needs ~50 Hz accuracy), 25 kHz otherwise */
    if (m == DSP_MODE_RS41) {
        set_rate(1152000); /* need the full 1.15 MHz view to find the sonde */
        s_step_i = 2; /* 1 kHz */
        /* sondes live at 400-406 MHz; jump there if we're elsewhere */
        if (s_center < 400000000u || s_center > 406000000u) retune(403000000);
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

/* ---- band-plan menu --------------------------------------------------------
 * s_menu: 0 = closed, 1 = plan list, 2 = USER-plan editor. While a menu is
 * open it owns the whole screen and all touches; the normal controls are
 * unreachable until it is closed. */

static int s_menu;

/* Plan list rows: 4 plans + EDIT USER PLAN + CLOSE. */
#define M1_ROWS      6
#define M1_ROW_Y0    48
#define M1_ROW_PITCH 36
#define M1_ROW_H     30
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

static void menu_close(void)
{
    s_menu = 0;
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0)); /* clear the panel/waterfall area the menu drew over */
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
            box_t r = {M1_ROW_X, M1_ROW_Y0 + i * M1_ROW_PITCH, M1_ROW_W, M1_ROW_H};
            bool sel = (i < 4) && bandplan_active() == i;
            rect(r.x, r.y, r.w, r.h, sel ? RGB565(30, 110, 75) : COL_BTN);
            rect(r.x, r.y, r.w, 2, COL_BTN_HI);
            text(r.x + 12, r.y + 8, 2, COL_TEXT, (i < 4) ? bandplan_name(i) : (i == 4 ? "EDIT USER PLAN" : "CLOSE"));
            if (i < 4) {
                plan_info(i, b, sizeof b);
                text(r.x + 300, r.y + 11, 1, RGB565(170, 180, 200), b);
            }
        }
        return;
    }
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
    if (s_menu == 1) {
        for (int i = 0; i < M1_ROWS; i++) {
            box_t r = {M1_ROW_X, M1_ROW_Y0 + i * M1_ROW_PITCH, M1_ROW_W, M1_ROW_H};
            if (hit(&r, x, y)) {
                if (i < 4) { bandplan_select(i); menu_close(); }
                else if (i == 4) s_menu = 2;
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
    static bool tap_moved;
    static uint32_t drag_f0;
    static int grace = 0;    /* ignore touches briefly after boot */
    static int run = 0;      /* consecutive raw-pressed samples (debounce) */
    bool changed = false;

    esp_lcd_touch_point_data_t p;
    int n = bsp_tab5_touch_read(&p, 1);

    /* ~1.2 s startup grace: the touch controller reports stray points right after
     * init that would otherwise "tap" a control. */
    if (grace < 150) { grace++; prev = false; run = 0; return false; }

    /* Debounce: a press must persist for 2 polls (~16 ms) before it counts,
     * rejecting single-sample glitches. */
    run = (n > 0) ? run + 1 : 0;
    bool pressed = run >= 2;
    int lx = 0, ly = 0;
    if (pressed) {
        lx = (PANEL_H - p.y) / SCALE;
        ly = p.x / SCALE;
        if (lx < 0) lx = 0;
        if (lx >= LW) lx = LW - 1;
        if (ly < 0) ly = 0;
        if (ly >= LH) ly = LH - 1;
    }
    bool edge = pressed && !prev;

    if (s_menu) { /* an open menu owns the screen and all touches */
        if (edge) changed = menu_touch(lx, ly);
        prev = pressed;
        return changed;
    }

    if (pressed && ly < CTRL_Y0) {
        /* drag the spectrum/waterfall to scan */
        if (edge) { dragging = true; drag_x0 = lx; drag_f0 = s_center; tap_moved = false; tap_y = ly; }
        else if (dragging) {
            if (lx - drag_x0 > 6 || drag_x0 - lx > 6) tap_moved = true;
            int64_t df = -(int64_t)(lx - drag_x0) * DRAG_HZ_PER_PX;
            retune((int64_t)drag_f0 + df);
            s_label = "MANUAL";
            changed = true;
        }
    } else if (edge) {
        dragging = false;
        if (hit(&B_TDN, lx, ly)) { retune((int64_t)s_center - STEPS[s_step_i]); changed = true; }
        else if (hit(&B_TUP, lx, ly)) { retune((int64_t)s_center + STEPS[s_step_i]); changed = true; }
        else if (hit(&B_STEP, lx, ly)) { s_step_i = (s_step_i + 1) % N_STEPS; changed = true; }
        else if (hit(&B_GAIN, lx, ly)) {
            s_gain_i = (s_gain_i + 1) % N_GAINS;
            sdr_set_gain(GAINS[s_gain_i]);
            changed = true;
        } else if (hit(&B_RS41, lx, ly)) {
            if (s_mode == DSP_MODE_RS41) { snap_to_peak(); s_label = "SNAP"; } /* second press: centre on the peak */
            else mode_button(DSP_MODE_RS41);
            changed = true;
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
        /* a tap (no movement) on the WSPR text panel runs the decoder self-test */
        if (dragging && !tap_moved && s_mode == DSP_MODE_WSPR && tap_y >= WF_Y0) { wspr_selftest(); changed = true; }
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

    draw_controls();
    draw_header();
    present();

    const float *last = NULL;
    int bins = dsp_spectrum_bins();
    int meter_div = 0;
    int last_psk = -1;
    while (1) {
        bool dirty = handle_touch();
        if (bandplan_tick()) dirty = true; /* scheduler: apply the slot for the current hour */
        if (s_menu) { /* an open menu owns the screen; the SDR keeps running underneath */
            if (dirty) { draw_menu(); present(); }
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        int ps = psk_status(); /* PSK box follows WiFi/SNTP state by itself */
        if (ps != last_psk) { last_psk = ps; dirty = true; }
        const float *spec = dsp_spectrum();
        bool newspec = spec && spec != last;
        if (newspec) {
            last = spec;
            draw_spectrum(spec, bins);
            if (++meter_div >= 4) { meter_div = 0; draw_header(); }
        }
        if (dirty) { draw_controls(); draw_header(); }
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

    bandplan_init();
    bandplan_set_apply_cb(plan_apply);

    ESP_LOGI(TAG, "landscape UI %dx%d (PPA x%d)", LW, LH, SCALE);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 6, NULL, 0);
}
