/*
 * Tab5-SDR — standalone RTL-SDR receiver on the M5Stack Tab5 (ESP32-P4).
 *
 * Stage 1: BSP bring-up + RGB565 test pattern (panel proof).
 * Stage 2: USB host power + librtlsdr enumeration (see sdr.c).
 * Stage 3: stream IQ into a PSRAM ring buffer (see sdr.c).
 * Stage 4: FFT spectrum + waterfall (see dsp.c).
 *
 * app_main starts the UI and network independently of RTL-SDR enumeration.
 * USB SDR enumeration runs in its own FreeRTOS task.
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "bsp_tab5.h"
#include "sdr.h"
#include "dsp.h"
#include "ft8.h"
#include "ft4.h"
#include "wspr.h"
#include "audio.h"
#include "ui.h"
#include "wifi_net.h"
#include "web_mirror.h"
#include "psk.h"
#include "lora_aprs.h"
#include "rs41_log.h"
#include "si5351.h"

static const char *TAG = "tab5-sdr";

/* Tab5 panel: 720x1280, RGB565 (see bsp_tab5.c). */
#define PANEL_W 720
#define PANEL_H 1280

#define RGB565(r, g, b) \
    ((uint16_t)(((((r) >> 3) & 0x1F) << 11) | ((((g) >> 2) & 0x3F) << 5) | (((b) >> 3) & 0x1F)))

static void draw_test_pattern(void)
{
    uint16_t *f0 = (uint16_t *)bsp_tab5_display_get_frame_buffer(0);
    uint16_t *f1 = (uint16_t *)bsp_tab5_display_get_frame_buffer(1);

    ESP_LOGI(TAG, "LCD framebuffer 0: %p", f0);
    ESP_LOGI(TAG, "LCD framebuffer 1: %p", f1);

    if (!f0) {
        ESP_LOGE(TAG, "Framebuffer 0 is NULL");
        return;
    }

    static const uint16_t bars[8] = {
        RGB565(0, 0, 0), RGB565(0, 0, 255),
        RGB565(0, 255, 0), RGB565(0, 255, 255),
        RGB565(255, 0, 0), RGB565(255, 0, 255),
        RGB565(255, 255, 0), RGB565(255, 255, 255)
    };

    for (int y = 0; y < PANEL_H; y++) {
        uint16_t *row = f0 + (size_t)y * PANEL_W;
        for (int x = 0; x < PANEL_W; x++) {
            row[x] = bars[(x * 8) / PANEL_W];
        }
    }

    bsp_tab5_display_flush(0);
    ESP_LOGI(TAG, "LCD test pattern sent");
}

/* sdr_open() waits for the dongle. Never call it from app_main():
 * otherwise the entire UI and Wi-Fi startup would wait for USB hardware.
 * Do not draw directly here: the UI owns the framebuffer after ui_start(). */
static void sdr_usb_task(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "RTL-SDR: waiting for USB dongle (UI remains available)");

    sdr_open();
    ESP_LOGW(TAG, "RTL-SDR: dongle opened");

    if (!sdr_stream_start(SDR_DEFAULT_CENTER_FREQ, SDR_DEFAULT_SAMPLE_RATE)) {
        ESP_LOGE(TAG, "RTL-SDR: unable to start IQ stream; UI remains active");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "RTL-SDR: IQ stream started");
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* Quiet normal operation; keep warnings and errors for diagnostics. */
    esp_log_level_set("*", ESP_LOG_WARN);
esp_log_level_set("SI5351", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Tab5-SDR boot — free heap %" PRIu32 ", PSRAM %u",
             esp_get_free_heap_size(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_ERROR_CHECK(bsp_tab5_init(&(bsp_tab5_config_t){
        .display.fb_num = 2, /* double buffer: PPA writes one while the other scans out */
        .touch.interrupt = true,
        .wifi.enable = false,
        .bluetooth.enable = false,
    }));
    /* Si5351 calibration is stored in NVS. Initialise it before Si5351. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(nvs_err));
        /* Do not erase NVS automatically: other saved settings may be present. */
        return;
    }

    // ===== Si5351 initialization =====
    ESP_ERROR_CHECK(si5351_init());
    /* Do not start the continuous 10 MHz calibration carrier at boot.
     * GPIO47 must remain LOW (RX) until an actual FT8 transmission. */

    bsp_tab5_display_set_brightness(80);
    draw_test_pattern();

    rs41_log_start(); /* Optional SD; failure does not stop SDR reception. */

    /* USB-A 5V VBUS is off by default — power the dongle, then enumerate. */
    bsp_tab5_set_usb_host_power(true);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* No blocking SDR enumeration here: boot must work without the dongle. */

    /* Speaker up, then demod + landscape UI take over the screen. */
    audio_init();
    dsp_start();
    /* FT4 decoder starts lazily when FT4 mode is selected, to preserve USB heap. */
    ft8_start(); /* FT8 decoder task (idle until FT8 mode feeds it audio) */
    wspr_start(); /* WSPR decoder task (likewise) */
    lora_aprs_init();
    ui_start();

    /* WiFi (P4 ← C6 via esp_hosted) + SNTP, then the PSK Reporter uploader.
     * Non-fatal without a network: the receiver works offline. */
    wifi_net_init();
    web_mirror_load_settings();
    web_mirror_start();
    psk_init();

    /* Enumerate the USB receiver asynchronously. This task can wait for
     * sdr_open() without preventing Tab5 startup. */
    if (xTaskCreate(sdr_usb_task, "sdr_usb", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "RTL-SDR: could not create USB task");
    }

    /* No periodic serial heartbeat. Keep app_main alive. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
