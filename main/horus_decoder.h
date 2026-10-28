#include <stdbool.h>
#pragma once
void horus_decoder_start(void);
void horus_decoder_reset(void);
void horus_decoder_feed_iq(float i,float q);
void horus_decoder_status(char *dst,unsigned size);

/* Snapshot of the latest CRC-valid Horus Binary v1 or v2 telemetry. */
typedef struct {
    bool valid;
    char callsign[16]; /* V3: callsign directly in packet */
    bool v3_has_temperature, v3_has_battery;
    unsigned v3_battery_mv;
    unsigned version;
    unsigned payload_id, sequence, hour, minute, second;
    float latitude, longitude;
    unsigned altitude, speed, satellites, battery_raw;
    int temperature;
} horus_v1_telemetry_t;
bool horus_decoder_get_telemetry(horus_v1_telemetry_t *out);

void horus_decoder_raw(char *dst, unsigned size, unsigned *version, unsigned *length, bool *parsed);
