/* SX1278 LoRa APRS receiver on external M3 GPIO header. ESP-IDF SPI master.
 * Default EU 433.775 MHz / SF12 / BW125 / CR4/5 / explicit header / CRC.
 * Adjust these parameters to match your LoRa APRS network.
 */
#include "lora_aprs.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN_SCK 5
#define PIN_MOSI 18
#define PIN_MISO 19
#define PIN_CS 45
#define PIN_RST 4
#define PIN_DIO0 2
#define LORA_FREQ_HZ 433775000UL
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_FRF_MSB 0x06
#define REG_PA_CONFIG 0x09
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_MODEM_CONFIG1 0x1d
#define REG_MODEM_CONFIG2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_MODEM_CONFIG3 0x26
#define REG_SYNC_WORD 0x39
#define REG_VERSION 0x42
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define IRQ_RX_DONE 0x40
#define IRQ_CRC_ERR 0x20
static const char *TAG = "lora_aprs";
static spi_device_handle_t dev;
static volatile bool active;
static volatile uint32_t revision;
static char status_line[192] = "LoRa APRS: standby";
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static lora_aprs_packet_t packets[LORA_APRS_HISTORY];
static uint32_t packet_count;
int lora_aprs_packets(lora_aprs_packet_t *out, int max_packets) {
    if (!out || max_packets <= 0) return 0;
    if (max_packets > LORA_APRS_HISTORY) max_packets = LORA_APRS_HISTORY;
    portENTER_CRITICAL(&mux);
    int n = packet_count < (uint32_t)max_packets ? (int)packet_count : max_packets;
    for (int i = 0; i < n; ++i)
        out[i] = packets[(packet_count - 1 - (uint32_t)i) % LORA_APRS_HISTORY];
    portEXIT_CRITICAL(&mux);
    return n;
}
uint32_t lora_aprs_packet_count(void) {
    portENTER_CRITICAL(&mux);
    uint32_t n = packet_count;
    portEXIT_CRITICAL(&mux);
    return n;
}

static uint8_t reg_read(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7f, 0}, rx[2] = {0};
    spi_transaction_t t = {.length=16, .tx_buffer=tx, .rx_buffer=rx};
    if (spi_device_transmit(dev, &t) != ESP_OK) return 0;
    return rx[1];
}
static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg | 0x80, val};
    spi_transaction_t t = {.length=16, .tx_buffer=tx};
    spi_device_transmit(dev, &t);
}
static void publish(const char *line) {
    portENTER_CRITICAL(&mux);
    snprintf(status_line, sizeof status_line, "%s", line);
    revision++;
    portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG, "%s", line);
}
void lora_aprs_status(char *out, unsigned size) {
    if (!out || !size) return;
    portENTER_CRITICAL(&mux);
    snprintf(out, size, "%s", status_line);
    portEXIT_CRITICAL(&mux);
}
uint32_t lora_aprs_revision(void) { return revision; }
bool lora_aprs_enabled(void) { return active; }
void lora_aprs_enable(bool enable) {
    if (!dev) { active = false; publish("LoRa APRS: RA-02 not found"); return; }
    active = enable;
    reg_write(REG_OPMODE, enable ? 0x85 : 0x81); /* LoRa RX continuous / standby */
    if (enable) publish("LoRa APRS: listening 433.775 SF12 BW125");
    else publish("LoRa APRS: standby");
}
static void task(void *arg) {
    (void)arg;
    while (1) {
        if (active) {
            uint8_t irq = reg_read(REG_IRQ_FLAGS);
            if (irq & IRQ_RX_DONE) {
                reg_write(REG_IRQ_FLAGS, irq);
                if (!(irq & IRQ_CRC_ERR)) {
                    uint8_t n = reg_read(REG_RX_NB_BYTES);
                    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_RX_CURRENT_ADDR));
                    char line[192];
                    unsigned j = 0;
                    uint8_t first[3] = {0};
                    for (unsigned i = 0; i < n; i++) {
                        uint8_t c = reg_read(REG_FIFO);
                        if (i < 3) first[i] = c;
                        /* LoRa APRS TNC payload prefix: 3C FF 01 */
                        if (i < 3) continue;
                        if (j + 1 < sizeof line) line[j++] = (c >= 32 && c <= 126) ? (char)c : '.';
                    }
                    line[j] = 0;
                    if (n >= 3 && first[0] == 0x3c && first[1] == 0xff && first[2] == 0x01) {
                        int8_t raw_snr = (int8_t)reg_read(REG_PKT_SNR_VALUE);
                        float snr = (float)raw_snr / 4.0f;
                        int rssi = -164 + (int)reg_read(REG_PKT_RSSI_VALUE);
                        if (snr < 0) rssi += (int)snr;
                        portENTER_CRITICAL(&mux);
                        lora_aprs_packet_t *p = &packets[packet_count % LORA_APRS_HISTORY];
                        snprintf(p->text, sizeof p->text, "%s", line);
                        p->rssi_dbm = rssi;
                        p->snr_db = snr;
                        p->tick_ms = pdTICKS_TO_MS(xTaskGetTickCount());
                        packet_count++;
                        portEXIT_CRITICAL(&mux);
                        publish(line);
                    }
                    else {
                        char msg[192];
                        snprintf(msg, sizeof msg, "LoRa packet (%u B, non-APRS header)", (unsigned)n);
                        publish(msg);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
void lora_aprs_init(void) {
    spi_bus_config_t bus = {.mosi_io_num=PIN_MOSI, .miso_io_num=PIN_MISO,
                            .sclk_io_num=PIN_SCK, .quadwp_io_num=-1, .quadhd_io_num=-1,
                            .max_transfer_sz=256};
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) { publish("LoRa APRS: SPI bus init failed"); return; }
    spi_device_interface_config_t cfg = {.clock_speed_hz=2000000, .mode=0,
                                          .spics_io_num=PIN_CS, .queue_size=1};
    err = spi_bus_add_device(SPI2_HOST, &cfg, &dev);
    if (err != ESP_OK) { publish("LoRa APRS: SPI device failed"); return; }
    gpio_config_t g = {.pin_bit_mask=1ULL << PIN_RST, .mode=GPIO_MODE_OUTPUT};
    gpio_config(&g);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t ver = reg_read(REG_VERSION);
    if (ver != 0x12) { dev = NULL; publish("LoRa APRS: SX1278 not detected (expect 0x12)"); return; }
    reg_write(REG_OPMODE, 0x80); /* LoRa sleep */
    reg_write(REG_OPMODE, 0x81); /* LoRa standby */
    uint64_t frf = ((uint64_t)LORA_FREQ_HZ << 19) / 32000000ULL;
    reg_write(REG_FRF_MSB, (frf >> 16) & 255);
    reg_write(REG_FRF_MSB+1, (frf >> 8) & 255);
    reg_write(REG_FRF_MSB+2, frf & 255);
    reg_write(REG_MODEM_CONFIG1, 0x72); /* BW125 CR4/5 explicit */
    reg_write(REG_MODEM_CONFIG2, 0xc4); /* SF12, CRC on */
    reg_write(REG_MODEM_CONFIG3, 0x0c); /* AGC and low data rate optimisation */
    reg_write(REG_PREAMBLE_MSB, 0);
    reg_write(REG_PREAMBLE_LSB, 8);
    reg_write(REG_SYNC_WORD, 0x12);
    reg_write(REG_PA_CONFIG, 0x00); /* receiver only */
    reg_write(REG_IRQ_FLAGS, 0xff);
    publish("LoRa APRS: SX1278 ready");
    xTaskCreate(task, "lora_aprs", 4096, NULL, 4, NULL);
}
