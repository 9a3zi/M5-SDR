/* Horus Binary V3: CRC-validated packet bytes -> ASN.1 UPER telemetry. */
#include "horus_v3.h"
#include "horusbinaryv3/Telemetry.h"
#include "horusbinaryv3/per_decoder.h"
#include "horusbinaryv3/asn_application.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool horus_v3_parse(const char *hex, horus_v1_telemetry_t *out) {
    if (!hex || !out) return false;
    size_t len = strlen(hex);
    if (len != 64 && len != 96 && len != 128 && len != 192 && len != 256) return false;
    unsigned char bytes[128];
    for (size_t i = 0; i < len / 2; ++i) {
        int a = nibble(hex[i*2]), b = nibble(hex[i*2+1]);
        if (a < 0 || b < 0) return false;
        bytes[i] = (unsigned char)((a << 4) | b);
    }
    /* V3 stores CRC16 in the first two bytes. The modem already validated it. */
    Telemetry_t *t = NULL;
    asn_dec_rval_t rc = uper_decode_complete(NULL, &asn_DEF_Telemetry,
                                              (void **)&t, bytes + 2, len/2 - 2);
    bool ok = false;
    if (rc.code == RC_OK && t && t->payloadCallsign.buf &&
        t->payloadCallsign.size > 0 && t->payloadCallsign.size <= 15 &&
        t->timeOfDaySeconds >= -1 && t->timeOfDaySeconds <= 86400 &&
        t->latitude >= -9000000 && t->latitude <= 9000000 &&
        t->longitude >= -18000000 && t->longitude <= 18000000) {
        horus_v1_telemetry_t v = {0};
        v.valid = true; v.version = 3;
        v.sequence = (unsigned)t->sequenceNumber;
        v.latitude = (float)t->latitude / 100000.0f;
        v.longitude = (float)t->longitude / 100000.0f;
        v.altitude = t->altitudeMeters < 0 ? 0u : (unsigned)t->altitudeMeters;
        if (t->timeOfDaySeconds >= 0) {
            unsigned sec = (unsigned)t->timeOfDaySeconds;
            v.hour = sec / 3600; v.minute = (sec / 60) % 60; v.second = sec % 60;
        }
        if (t->velocityHorizontalKilometersPerHour) v.speed = (unsigned)*t->velocityHorizontalKilometersPerHour;
        if (t->gnssSatellitesVisible) v.satellites = (unsigned)*t->gnssSatellitesVisible;
        if (t->temperatureCelsius_x10) {
            long *temp = t->temperatureCelsius_x10->external ? t->temperatureCelsius_x10->external : t->temperatureCelsius_x10->internal;
            if (temp) { v.temperature = (int)(*temp / 10); v.v3_has_temperature = true; }
        }
        if (t->milliVolts && t->milliVolts->battery) {
            v.v3_battery_mv = (unsigned)*t->milliVolts->battery;
            v.v3_has_battery = true;
        }
        memcpy(v.callsign, t->payloadCallsign.buf, t->payloadCallsign.size);
        v.callsign[t->payloadCallsign.size] = '\0';
        *out = v;
        ok = true;
    }
    if (t) ASN_STRUCT_FREE(asn_DEF_Telemetry, t);
    return ok;
}
