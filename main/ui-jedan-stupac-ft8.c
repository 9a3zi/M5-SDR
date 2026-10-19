/*
 * ui.c — landscape touch UI for Tab5-SDR.
 *
 * Rendered into a 640x360 landscape RGB565 buffer (LBUF); the P4 PPA scales x2
 * and rotates 90° onto the 720x1280 panel (half-res keeps the PPA cheap).
 * Touch maps back: lx = (1280 - panel_y)/2,  ly = panel_x/2.
 *
 * Controls: drag spectrum/waterfall to scan fast; [-1][-.1][+.1][+1] tune;
 * mode row WFM/NFM/AM/USB/LSB/CW/FT8, RS41 button (press again = centre on peak); HF/VHF/UHF band buttons; volume +/- ;
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
#include "audio.h"
#include "bookmarks.h"

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
static const uint32_t STEPS[] = {100, 500, 1000, 5000, 6250, 12500, 25000, 100000, 1000000};
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

/* RS41 mode: the waterfall area becomes a telemetry panel. */
static void draw_rs41_panel(void)
{
    static uint32_t last_seq;
    static TickType_t last_tick;
    rs41_info_t in;
    bool have = rs41_get_info(&in);
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

/* FT8 mode: the waterfall area shows the list of decoded messages, newest first. */
static void draw_ft8_panel(void)
{
    ft8_msg_t msgs[9];
    ft8_status_t st;
    int n = ft8_get_messages(msgs, 9, &st); /* oldest first */
    rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0));

    char b[100];
    int y = WF_Y0 + 3;
    if (st.locked)
        snprintf(b, sizeof b, "FT8  SLOT SYNC OK   T %04.1f S   SLOTS %u   MSGS %u", (double)st.slot_pos,
                 (unsigned)st.slots, (unsigned)st.total);
    else
        snprintf(b, sizeof b, "FT8  SCANNING FOR THE SLOT TIMING   MSGS %u", (unsigned)st.total);
    text(6, y, 1, st.locked ? RGB565(60, 220, 140) : RGB565(255, 190, 60), b);
    y += 11;

    if (n == 0) {
        text(6, y, 1, RGB565(170, 170, 170), "NO MESSAGES YET. TUNE TO AN FT8 FREQUENCY IN USB,");
        text(6, y + 10, 1, RGB565(170, 170, 170), "E.G. 14.074 MHZ (20 M). THE FIRST DECODE SETS THE SLOT TIMING.");
        return;
    }
    uint16_t newest_slot = msgs[n - 1].slot;
    for (int i = n - 1; i >= 0 && y < WF_Y0 + WF_H - 8; i--, y += 10) {
        snprintf(b, sizeof b, "%4d %+3d %+4.1f %s", (int)msgs[i].freq, (int)msgs[i].snr, (double)msgs[i].dt, msgs[i].text);
        uint16_t col = (msgs[i].slot == newest_slot) ? RGB565(235, 235, 235) : RGB565(120, 120, 120);
        if (msgs[i].slot == newest_slot && msgs[i].text[0] == 'C' && msgs[i].text[1] == 'Q')
            col = RGB565(255, 220, 0); /* CQ calls */
        text(6, y, 1, col, b);
    }
}

static void draw_spectrum(const float *spec, int bins)
{
    const bool rs = (s_mode == DSP_MODE_RS41 || s_mode == DSP_MODE_FT8); /* mode with a text panel */
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
}

/* ---- control band -------------------------------------------------------- */

typedef struct { int x, y, w, h; } box_t;
static const box_t B_TDN  = {  6, 220, 56, 44};  /* tune down one step */
static const box_t B_STEP = { 66, 220, 96, 44};  /* cycle step size    */
static const box_t B_TUP  = {166, 220, 56, 44};  /* tune up one step   */
static const box_t B_RS41 = {256, 220, 92, 44};  /* RS41 mode; pressed again in RS41 mode = centre on the strongest peak */
static const box_t B_GAIN = {352, 220, 48, 44};  /* tuner gain: AGC / manual steps */
static const box_t B_HF   = {  6, 272, 108, 44};
static const box_t B_VHF  = {118, 272, 108, 44};
static const box_t B_UHF  = {230, 272, 108, 44};
/* Mode row: WFM NFM AM USB LSB CW FT8 — 7 buttons, 52 px wide, 3 px gap (RS41 has its own button). */
static const uint8_t MODE_ROW[] = {DSP_MODE_WFM, DSP_MODE_NFM, DSP_MODE_AM, DSP_MODE_USB, DSP_MODE_LSB, DSP_MODE_CW, DSP_MODE_FT8};
#define N_MODE_ROW (int)(sizeof(MODE_ROW) / sizeof(MODE_ROW[0]))
#define MODE_BTN_W 52
#define MODE_BTN_X(i) (6 + (i) * (MODE_BTN_W + 3))
#define MODE_BTN_Y 322
#define MODE_BTN_H 36
static const box_t B_VDN  = {404, 226, 46, 44};
static const box_t B_VUP  = {540, 226, 46, 44};
static const box_t B_SDN  = {404, 300, 46, 44};
static const box_t B_SUP  = {540, 300, 46, 44};

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
    for (int i = 0; i < N_MODE_ROW; i++) {
        int m = MODE_ROW[i];
        box_t b = { MODE_BTN_X(i), MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H };
        draw_box(&b, dsp_mode_name(m), 2, (m == s_mode) ? mode_color(m) : COL_BTN);
    }
    draw_box(&B_HF, "HF", 2, COL_BTN);
    draw_box(&B_VHF, "VHF", 2, COL_BTN);
    draw_box(&B_UHF, "UHF", 2, COL_BTN);

    char v[8];
    text(404, 212, 2, COL_TEXT, "VOL");
    draw_box(&B_VDN, "-", 3, COL_BTN);
    draw_box(&B_VUP, "+", 3, COL_BTN);
    snprintf(v, sizeof v, "%d", s_volume);
    text(470, 238, 3, COL_LINE, v);

    text(404, 286, 2, COL_TEXT, "SQL");
    draw_box(&B_SDN, "-", 3, COL_BTN);
    draw_box(&B_SUP, "+", 3, COL_BTN);
    snprintf(v, sizeof v, "%d", s_squelch);
    text(466, 312, 3, COL_LINE, v);
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

/* Standard FT8 dial frequencies (USB): 80, 40, 30, 20, 17, 15, 12, 10 m. */
static const uint32_t FT8_FREQS[] = {3573000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000};
#define N_FT8_FREQS (int)(sizeof(FT8_FREQS) / sizeof(FT8_FREQS[0]))
static int s_ft8_i = 3; /* 20 m */

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
    if ((s_mode == DSP_MODE_RS41 || s_mode == DSP_MODE_FT8) && mode != DSP_MODE_RS41 && mode != DSP_MODE_FT8)
        rect(0, WF_Y0, LW, WF_H, RGB565(0, 0, 0)); /* clear the text panel */
    s_mode = mode;
    dsp_set_mode(mode);
}

/* A mode button was pressed: select the mode and its sensible defaults. */
static void mode_button(int m)
{
    const bool was_ft8 = (s_mode == DSP_MODE_FT8);
    set_mode(m);
    s_label = "MANUAL";
    /* sensible default step: 100 Hz for SSB/CW (SSB needs ~50 Hz accuracy), 25 kHz otherwise */
    if (m == DSP_MODE_RS41) {
        set_rate(1152000); /* need the full 1.15 MHz view to find the sonde */
        s_step_i = 2; /* 1 kHz */
        /* sondes live at 400-406 MHz; jump there if we're elsewhere */
        if (s_center < 400000000u || s_center > 406000000u) retune(403000000);
    } else if (m == DSP_MODE_FT8) {
        set_rate(288000); /* narrow 96 kHz view, like HF */
        set_step_hz(100);
        if (was_ft8) { /* pressed again: next FT8 band */
            s_ft8_i = (s_ft8_i + 1) % N_FT8_FREQS;
            retune(FT8_FREQS[s_ft8_i]);
        } else {
            bool near = false; /* already on an FT8 frequency? keep it */
            for (int i = 0; i < N_FT8_FREQS; i++) {
                int64_t d = (int64_t)s_center - (int64_t)FT8_FREQS[i];
                if (d > -3000 && d < 3000) { s_ft8_i = i; near = true; break; }
            }
            if (!near) retune(FT8_FREQS[s_ft8_i]);
        }
        s_label = "FT8";
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

static bool handle_touch(void)
{
    static bool prev = false, dragging = false;
    static int drag_x0;
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

    if (pressed && ly < CTRL_Y0) {
        /* drag the spectrum/waterfall to scan */
        if (edge) { dragging = true; drag_x0 = lx; drag_f0 = s_center; }
        else if (dragging) {
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

    if (!pressed) dragging = false;
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
    draw_controls();
    draw_header();
    present();

    const float *last = NULL;
    int bins = dsp_spectrum_bins();
    int meter_div = 0;
    while (1) {
        bool dirty = handle_touch();
        const float *spec = dsp_spectrum();
        bool newspec = spec && spec != last;
        if (newspec) {
            last = spec;
            draw_spectrum(spec, bins);
            if (++meter_div >= 4) { meter_div = 0; draw_header(); }
        }
        if (dirty) { draw_controls(); draw_header(); }
        if (newspec || dirty) present();
        vTaskDelay(pdMS_TO_TICKS(8));
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

    ESP_LOGI(TAG, "landscape UI %dx%d (PPA x%d)", LW, LH, SCALE);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 6, NULL, 0);
}
