#pragma once
#include <stdint.h>
#include <stddef.h>
/* DSP task owns decoder; UI only reads snapshots. */
#define APRS_HISTORY 5
#define APRS_LINE 112
typedef struct { char text[APRS_LINE]; } aprs_packet_t;
void aprs_reset(void);
void aprs_feed(float sample); /* 48 kHz, FM discriminator output */
uint32_t aprs_revision(void);
uint32_t aprs_packet_count(void);
uint32_t aprs_crc_errors(void);
int aprs_packets(aprs_packet_t *out, int cap);

uint32_t aprs_flags(void);
uint32_t aprs_transitions(void);
int aprs_quality_pct(void); /* smoothed mark/space contrast, 0..100 */
