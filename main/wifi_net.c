/*
 * wifi_net.c — see wifi_net.h.
 *
 * On the ESP32-P4 the WiFi radio lives in an ESP32-C6 co-processor reachable
 * through esp_hosted (SDIO). The espressif/esp_wifi_remote component provides
 * the normal esp_wifi API on top of that link, so this file is plain IDF
 * station code. The C6 must run the esp-hosted-mcu firmware (the Tab5 ships
 * with it; if WiFi never connects, reflash the C6 per the board docs).
 */
#include "wifi_net.h"

#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "net_cfg.h"

static const char *TAG = "wifi";

static EventGroupHandle_t s_ev;
#define EV_GOT_IP BIT0
static volatile bool s_time_ok;
static volatile bool s_sntp_started;
static char s_ip[16] = "0.0.0.0";

static void sntp_cb(struct timeval *tv)
{
    s_time_ok = true;
    ESP_LOGI(TAG, "time synced: %s", ctime(&tv->tv_sec));
}

static void net_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_time_ok = false;
        ESP_LOGW(TAG, "disconnected, retrying…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof s_ip, IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "got IP %s", s_ip);
        xEventGroupSetBits(s_ev, EV_GOT_IP);
        if (!s_sntp_started) {
            s_sntp_started = true;
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            sntp_set_time_sync_notification_cb(sntp_cb);
            esp_sntp_init();
        }
    }
}

void wifi_net_init(void)
{
    /* NVS may already be initialised (hosted uses it too) — tolerate that. */
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK && err != ESP_ERR_NVS_NO_FREE_PAGES && err != ESP_ERR_NVS_NEW_VERSION_FOUND)
        ESP_LOGW(TAG, "nvs_flash_init: %s", esp_err_to_name(err));

    s_ev = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &net_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &net_evt, NULL));

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, NET_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, NET_WIFI_PASSWORD, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; /* open networks: use WIFI_AUTH_OPEN */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "connecting to '%s'…", NET_WIFI_SSID);
}

bool wifi_net_connected(void)
{
    return s_ev && (xEventGroupGetBits(s_ev) & EV_GOT_IP);
}

bool wifi_net_time_ok(void)
{
    /* the callback sets the flag; double-check the clock is past 2024 */
    return s_time_ok || time(NULL) > 1700000000;
}

const char *wifi_net_ip(void)
{
    return s_ip;
}
