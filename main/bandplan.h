/*
 * bandplan.h — time-of-day band scheduler ("band hopping") for FT8/WSPR monitoring.
 *
 * A plan is a list of slots: at slot start hour (local time) the receiver
 * retunes to the slot's dial frequency and switches the demod mode; it stays
 * there until the next slot. Built-in plans cover all FT8 / all WSPR bands
 * ordered by typical propagation; the USER plan (editable from the UI, stored
 * in NVS) can mix any FT8/WSPR bands, e.g. 40 m 08-11 h, 15 m 11-13 h, ...
 *
 * The scheduler needs a valid wall clock: ui.c calls bandplan_tick() about
 * once per second; slots are only applied once SNTP time is available
 * (wifi_net_time_ok). ui.c registers the retune callback with
 * bandplan_set_apply_cb().
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One schedule slot: from `hour` (local, 0..23) until the next slot starts,
 * listen to `band` (index into the dial-frequency table of `mode`). */
typedef struct {
    uint8_t hour; /* slot start hour, 0..23                       */
    uint8_t mode; /* DSP_MODE_FT8 or DSP_MODE_WSPR                */
    uint8_t band; /* index into the mode's dial-frequency table   */
} bp_slot_t;

/* Max slots of the editable USER plan (also the editor page size). */
#define BP_MAX_SLOTS 10

/* Plan ids for bandplan_select() / bandplan_active(). */
enum { BP_OFF = 0, BP_FT8 = 1, BP_WSPR = 2, BP_USER = 3, BP_N_PLANS };

void bandplan_init(void); /* load plan selection + USER plan from NVS, set TZ */

int bandplan_count(void);            /* BP_N_PLANS */
const char *bandplan_name(int plan); /* "OFF" / "FT8 PLAN" / "WSPR PLAN" / "USER PLAN" */
int bandplan_active(void);           /* current plan id (BP_OFF when disabled) */
void bandplan_select(int plan);      /* select + persist; applies the current slot now */

/* Dial-frequency tables (USB dial, Hz), shared with the UI presets. */
const uint32_t *bandplan_ft8_freqs(int *n);
const uint32_t *bandplan_wspr_freqs(int *n);
uint32_t bandplan_slot_freq(const bp_slot_t *s);

/* "40M" style band name for a dial frequency (buf must hold >= 5 chars). */
const char *bandplan_band_name(uint32_t freq_hz, char *buf, size_t n);

/* USER plan editing. Every change is persisted to NVS; if the USER plan is
 * armed it is re-applied at the next tick. */
int bandplan_user_n(void);
bp_slot_t bandplan_user_get(int i);
void bandplan_user_set(int i, const bp_slot_t *s); /* replace slot i (clamped) */
void bandplan_user_add(const bp_slot_t *s);        /* append (ignored when full) */
void bandplan_user_del(int i);                     /* remove slot i */
void bandplan_user_clear(void);

/* Wall-clock info for the status display. */
bool bandplan_time_ok(void);                     /* SNTP time available */
void bandplan_now_hm(int *hh, int *mm);          /* local time */
bool bandplan_current(bp_slot_t *out);           /* active slot of the armed plan */
int bandplan_current_until(void);                /* hour (0..24) the current slot ends, -1 if none */

/* Band-application hook: called in the UI task context whenever the active
 * slot changes (and once when a plan is selected). The callback retunes and
 * switches the demod mode. */
void bandplan_set_apply_cb(void (*cb)(int mode, uint32_t freq));

/* Call ~1×/s (throttled internally). Returns true when it applied a slot. */
bool bandplan_tick(void);

#ifdef __cplusplus
}
#endif
