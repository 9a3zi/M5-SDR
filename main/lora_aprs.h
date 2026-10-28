#pragma once
#include <stdbool.h>
#include <stdint.h>
void lora_aprs_init(void);
void lora_aprs_enable(bool enable);
bool lora_aprs_enabled(void);
uint32_t lora_aprs_revision(void);
void lora_aprs_status(char *out, unsigned size);

#define LORA_APRS_HISTORY 5
typedef struct {
    char text[192];
    int rssi_dbm;
    float snr_db;
    uint32_t tick_ms;
} lora_aprs_packet_t;
int lora_aprs_packets(lora_aprs_packet_t *out, int max_packets);
uint32_t lora_aprs_packet_count(void);
