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

/* Independent generic LoRa RX profile, saved in NVS. */
typedef struct { uint32_t freq_hz; uint8_t sf; uint32_t bw_hz; uint8_t sync_word; } lora_rx_config_t;
void lora_rx_get_config(lora_rx_config_t *out);
bool lora_rx_save_config(const lora_rx_config_t *cfg);
void lora_rx_enable(bool enable);
bool lora_rx_enabled(void);
