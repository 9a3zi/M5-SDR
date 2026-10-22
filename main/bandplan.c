/*
 * bandplan.c — see bandplan.h.
 *
 * Built-in plans (local time, ordered for typical propagation):
 *
 *   FT8  (10 slots, covers all 8 FT8 bands):
 *     00-03 80 m, 03-06 40 m, 06-08 30 m, 08-11 20 m, 11-13 17 m,
 *     13-15 15 m, 15-17 12 m, 17-19 10 m, 19-21 30 m, 21-24 40 m
 *   WSPR (12 slots, covers all 11 WSPR bands):
 *     00-02 160 m, 02-04 80 m, 04-06 60 m, 06-08 40 m, 08-10 30 m,
 *     10-12 20 m, 12-14 17 m, 14-16 15 m, 16-18 12 m, 18-20 10 m,
 *     20-22 6 m, 22-24 80 m
 *
 * On first boot the USER plan is prefilled with the classic example
 * (40 m 08-11 h, 15 m 11-13 h, 20 m 13-16 h) so the editor shows something
 * sensible; edit it from the PLAN menu.
 */
#include "bandplan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"

#include "dsp.h"
#include "wifi_net.h"
#include "net_cfg.h"

static const char *TAG = "bandplan";

#define NVS_NS "bplan"

/* ---- dial-frequency tables (USB dial, Hz) --------------------------------- */

static const uint32_t FT8_FREQS[] = {1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000};
static const uint32_t WSPR_FREQS[] = {1836600, 3568600, 5287200, 7038600, 10138700, 14095600,
                                      18104600, 21094600, 24924600, 28124600, 50293000};

const uint32_t *bandplan_ft8_freqs(int *n)
{
    *n = (int)(sizeof FT8_FREQS / sizeof FT8_FREQS[0]);
    return FT8_FREQS;
}

const uint32_t *bandplan_wspr_freqs(int *n)
{
    *n = (int)(sizeof WSPR_FREQS / sizeof WSPR_FREQS[0]);
    return WSPR_FREQS;
}

uint32_t bandplan_slot_freq(const bp_slot_t *s)
{
    int n;
    const uint32_t *t = (s->mode == DSP_MODE_WSPR) ? bandplan_wspr_freqs(&n) : bandplan_ft8_freqs(&n);
    if (s->band >= n) return t[0];
    return t[s->band];
}

const char *bandplan_band_name(uint32_t f, char *buf, size_t n)
{
    static const struct { uint32_t top; char nm[5]; } T[] = {
        { 2000000u, "160M"}, { 4000000u, "80M"}, { 5500000u, "60M"},
        { 8000000u, "40M"},  {12000000u, "30M"}, {15000000u, "20M"},
        {19000000u, "17M"},  {22000000u, "15M"}, {26000000u, "12M"},
        {30000000u, "10M"},  {60000000u, "6M"},
    };
    for (size_t i = 0; i < sizeof T / sizeof *T; i++)
        if (f < T[i].top) { snprintf(buf, n, "%s", T[i].nm); return buf; }
    snprintf(buf, n, "%uM", (unsigned)((300000000u + f / 2) / f)); /* ~ wavelength */
    return buf;
}

/* ---- built-in plans -------------------------------------------------------- */

/* FT8 band indices: 0=80 1=40 2=30 3=20 4=17 5=15 6=12 7=10 m.
 * WSPR band indices: 0=160 1=80 2=60 3=40 4=30 5=20 6=17 7=15 8=12 9=10 10=6 m. */
static const bp_slot_t PL_FT8[] = {
    { 0, DSP_MODE_FT8, 0}, { 3, DSP_MODE_FT8, 1}, { 6, DSP_MODE_FT8, 2},
    { 8, DSP_MODE_FT8, 3}, {11, DSP_MODE_FT8, 4}, {13, DSP_MODE_FT8, 5},
    {15, DSP_MODE_FT8, 6}, {17, DSP_MODE_FT8, 7}, {19, DSP_MODE_FT8, 2},
    {21, DSP_MODE_FT8, 1},
};
static const bp_slot_t PL_WSPR[] = {
    { 0, DSP_MODE_WSPR, 0}, { 2, DSP_MODE_WSPR, 1}, { 4, DSP_MODE_WSPR, 2},
    { 6, DSP_MODE_WSPR, 3}, { 8, DSP_MODE_WSPR, 4}, {10, DSP_MODE_WSPR, 5},
    {12, DSP_MODE_WSPR, 6}, {14, DSP_MODE_WSPR, 7}, {16, DSP_MODE_WSPR, 8},
    {18, DSP_MODE_WSPR, 9}, {20, DSP_MODE_WSPR, 10}, {22, DSP_MODE_WSPR, 1},
};

/* First-boot USER plan example: 40 m 08-11 h, 15 m 11-13 h, 20 m 13-16 h. */
static const bp_slot_t PL_USER_DEFAULT[] = {
    {8, DSP_MODE_FT8, 1}, {11, DSP_MODE_FT8, 5}, {13, DSP_MODE_FT8, 3},
};

/* ---- state ------------------------------------------------------------------ */

static bp_slot_t s_user[BP_MAX_SLOTS];
static int s_user_n;
static int s_active = BP_OFF;
static int s_applied = -1; /* (plan << 8) | slot index of the last applied slot */
static void (*s_cb)(int mode, uint32_t freq);

/* Slot active at `hour`: the latest-starting slot whose start hour is <= hour.
 * When nothing has started yet (hour before every slot) the schedule wraps at
 * midnight: the latest-starting slot overall stays active (its window crosses
 * midnight into the next day). Robust against any slot order and duplicate
 * hours (for duplicates the later slot in the list wins). */
static int slot_at(const bp_slot_t *s, int n, int hour)
{
    int cur = -1;
    for (int i = 0; i < n; i++) {
        if (s[i].hour > hour) continue;
        if (cur < 0 || s[i].hour >= s[cur].hour) cur = i;
    }
    if (cur >= 0) return cur;
    cur = 0; /* overnight continuation of the latest-starting slot */
    for (int i = 1; i < n; i++)
        if (s[i].hour >= s[cur].hour) cur = i;
    return cur;
}

static bool get_plan(int plan, const bp_slot_t **slots, int *n)
{
    switch (plan) {
    case BP_FT8:  *slots = PL_FT8;  *n = (int)(sizeof PL_FT8 / sizeof PL_FT8[0]);   return true;
    case BP_WSPR: *slots = PL_WSPR; *n = (int)(sizeof PL_WSPR / sizeof PL_WSPR[0]); return true;
    case BP_USER: *slots = s_user;  *n = s_user_n;                                   return true;
    default:      return false;
    }
}

/* ---- NVS --------------------------------------------------------------------- */

static void save_active(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "act", (uint8_t)s_active);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void user_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "user", s_user, (size_t)s_user_n * sizeof(bp_slot_t));
        nvs_commit(h);
        nvs_close(h);
    }
    if (s_active == BP_USER) s_applied = -1; /* the schedule changed: re-apply */
}

void bandplan_init(void)
{
    /* The scheduler works in local time; SNTP delivers UTC. */
    setenv("TZ", NET_TZ, 1);
    tzset();

    bool have_nvs = false;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        have_nvs = true;
        uint8_t a = BP_OFF;
        nvs_get_u8(h, "act", &a);
        if (a < BP_N_PLANS) s_active = a;
        size_t len = sizeof s_user;
        if (nvs_get_blob(h, "user", s_user, &len) == ESP_OK && len >= sizeof(bp_slot_t)) {
            s_user_n = (int)(len / sizeof(bp_slot_t));
            if (s_user_n > BP_MAX_SLOTS) s_user_n = BP_MAX_SLOTS;
        }
        nvs_close(h);
    }
    if (!have_nvs) { /* first boot: the example schedule (see file header) */
        s_user_n = (int)(sizeof PL_USER_DEFAULT / sizeof PL_USER_DEFAULT[0]);
        memcpy(s_user, PL_USER_DEFAULT, sizeof PL_USER_DEFAULT);
        user_save();
    }
    ESP_LOGI(TAG, "planner init: plan=%d, user slots=%d", s_active, s_user_n);
}

/* ---- selection / application --------------------------------------------------- */

int bandplan_count(void) { return BP_N_PLANS; }

const char *bandplan_name(int plan)
{
    switch (plan) {
    case BP_FT8:  return "FT8 PLAN";
    case BP_WSPR: return "WSPR PLAN";
    case BP_USER: return "USER PLAN";
    default:      return "OFF";
    }
}

int bandplan_active(void) { return s_active; }

static void apply_slot(int plan, int idx, const bp_slot_t *s)
{
    uint32_t f = bandplan_slot_freq(s);
    s_applied = (plan << 8) | idx;
    ESP_LOGI(TAG, "apply: %s %s @ %lu Hz (slot %d, plan %d)",
             dsp_mode_name(s->mode), bandplan_band_name(f, (char[8]){0}, 8),
             (unsigned long)f, idx, plan);
    if (s_cb) s_cb(s->mode, f);
}

void bandplan_select(int plan)
{
    if (plan < 0 || plan >= BP_N_PLANS) plan = BP_OFF;
    s_active = plan;
    save_active();
    s_applied = -1;
    bandplan_tick(); /* apply the current slot right away (with a valid clock) */
}

bool bandplan_tick(void)
{
    static uint32_t s_last_ms;
    uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    if (now_ms - s_last_ms < 900) return false; /* ~1 s cadence is plenty */
    s_last_ms = now_ms;

    if (s_active == BP_OFF || !s_cb || !bandplan_time_ok()) return false;
    const bp_slot_t *s;
    int n;
    if (!get_plan(s_active, &s, &n) || n == 0) return false;

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    int idx = slot_at(s, n, tm.tm_hour);
    if (((s_active << 8) | idx) == s_applied) return false;

    apply_slot(s_active, idx, &s[idx]);
    return true;
}

void bandplan_set_apply_cb(void (*cb)(int mode, uint32_t freq)) { s_cb = cb; }

/* ---- USER plan editing ------------------------------------------------------------ */

int bandplan_user_n(void) { return s_user_n; }

bp_slot_t bandplan_user_get(int i)
{
    if (i < 0 || i >= s_user_n) return (bp_slot_t){0, DSP_MODE_FT8, 0};
    return s_user[i];
}

void bandplan_user_set(int i, const bp_slot_t *s)
{
    if (i < 0 || i >= s_user_n) return;
    s_user[i] = *s;
    user_save();
}

void bandplan_user_add(const bp_slot_t *s)
{
    if (s_user_n >= BP_MAX_SLOTS) return;
    s_user[s_user_n++] = *s;
    user_save();
}

void bandplan_user_del(int i)
{
    if (i < 0 || i >= s_user_n) return;
    memmove(&s_user[i], &s_user[i + 1], (size_t)(s_user_n - i - 1) * sizeof(bp_slot_t));
    s_user_n--;
    user_save();
}

void bandplan_user_clear(void)
{
    s_user_n = 0;
    user_save();
}

/* ---- wall-clock info --------------------------------------------------------------- */

bool bandplan_time_ok(void) { return wifi_net_time_ok(); }

void bandplan_now_hm(int *hh, int *mm)
{
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    if (hh) *hh = tm.tm_hour;
    if (mm) *mm = tm.tm_min;
}

bool bandplan_current(bp_slot_t *out)
{
    const bp_slot_t *s;
    int n;
    if (s_active == BP_OFF || !bandplan_time_ok()) return false;
    if (!get_plan(s_active, &s, &n) || n == 0) return false;
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    if (out) *out = s[slot_at(s, n, tm.tm_hour)];
    return true;
}

int bandplan_current_until(void)
{
    const bp_slot_t *s;
    int n;
    if (s_active == BP_OFF || !bandplan_time_ok()) return -1;
    if (!get_plan(s_active, &s, &n) || n == 0) return -1;
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    int idx = slot_at(s, n, tm.tm_hour);
    return (idx + 1 < n) ? s[idx + 1].hour : s[0].hour + 24;
}
