/*
 * rs41.h — Vaisala RS41 radiosonde decoder (400–406 MHz, 4800 baud GFSK).
 *
 * Input : FM-discriminator samples at 48 kHz (10 samples per bit), fed one at
 *         a time by dsp.c while in DSP_MODE_RS41.
 * Output: decoded telemetry (serial, frame number, GPS position/velocity/time),
 *         readable from the UI task via rs41_get_info().
 *
 * Frame handling: header search (both polarities) → bit capture → de-whitening
 * → Reed-Solomon RS(255,231) x2 interleaved → per-block CRC16 → field parse.
 * Format constants (header, whitening mask, block offsets) follow the public
 * RS41 frame description; the Reed-Solomon / sync code here is an independent
 * implementation.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char id[9];             /* serial, e.g. "S1234567"                       */
    uint16_t frame_no;      /* sonde frame counter (1 frame per second)       */
    bool have_id;           /* status block CRC was good                      */
    bool have_pos;          /* position block CRC good + plausible altitude   */
    bool have_time;         /* GPS time block CRC good                        */
    bool encrypted;         /* RS41-SGM: block 0x80A7 present, payload is encrypted */

    double lat, lon;        /* degrees (WGS84)                                */
    double alt;             /* metres above ellipsoid                         */
    float vh, vd, vv;       /* horizontal speed m/s, heading deg, climb m/s   */
    uint8_t sats;           /* satellites used                                */

    uint16_t week;          /* GPS week                                       */
    uint32_t tow_ms;        /* GPS time of week, ms                           */
    int hh, mm, ss;         /* UTC time of day (GPS time minus leap seconds)  */

    uint32_t frames_ok;     /* frames with at least one valid block           */
    uint32_t frames_bad;    /* locked frames with no valid block              */
    int rs_fixed;           /* symbols corrected in the last frame, -1 = RS failed */
    uint8_t mk;             /* of 7 fixed marker bytes, how many were right before RS */
    uint8_t blk_ok;         /* blocks with a valid CRC in the last frame      */
    uint8_t nblk;           /* number of entries in btype                     */
    uint8_t btype[10];      /* types of the valid blocks, in order (diagnostics) */
    float eye;              /* header eye opening (rad/sample); ~0.6 for a clean sonde */
    uint32_t seq;           /* increments on every decoded frame              */
} rs41_info_t;

/* Forget sync state (call when entering RS41 mode). Telemetry is kept. */
void rs41_reset(void);

/* Feed one discriminator sample (rad/sample, 48 kHz). */
void rs41_feed(float d);

/* Copy the latest telemetry. Returns false until a frame has been decoded. */
bool rs41_get_info(rs41_info_t *out);

/* Decode one captured frame (still whitened, as received, starting with the
 * 8 header bytes). len must be 320 or 518. Updates `info`. Returns the number
 * of valid blocks found (0..3). Exposed for host tests. */
int rs41_decode_frame(const uint8_t *raw, int len, rs41_info_t *info);

#ifdef __cplusplus
}
#endif
