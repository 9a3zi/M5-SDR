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
#include "station_settings.h"
#include "nvs.h"

static const char *TAG = "wifi";

static EventGroupHandle_t s_ev;
#define EV_GOT_IP BIT0
static volatile bool s_time_ok;
static volatile bool s_sntp_started;
static char s_ip[16] = "0.0.0.0";
static uint8_t s_mode = 0; /* 0 STA, 1 AP, 2 AP+STA */
static bool s_started;
#define AP_SSID "M5-SDR"
#define AP_PASSWORD "a1b2c3d4"
static const char *s_ap_ip = "192.168.4.1";

static void load_mode(void)
{
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open("station", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "wifi_mode", &v);
        nvs_close(h);
    }
    s_mode = v <= 2 ? v : 0;
}

static void configure_wifi(void)
{
    station_settings_t st;
    station_settings_get(&st);
    wifi_config_t wc = {0};
    if (s_started) {
        esp_wifi_stop();
        s_started = false;
    }
    xEventGroupClearBits(s_ev, EV_GOT_IP);
    snprintf(s_ip, sizeof s_ip, "0.0.0.0");
    wifi_mode_t mode = s_mode == 1 ? WIFI_MODE_AP :
                       s_mode == 2 ? WIFI_MODE_APSTA : WIFI_MODE_STA;
    esp_err_t e = esp_wifi_set_mode(mode);
    if (e != ESP_OK) { ESP_LOGE(TAG, "set_mode: %s", esp_err_to_name(e)); return; }
    if (s_mode != 1) {
        strncpy((char *)wc.sta.ssid, st.ssid, sizeof wc.sta.ssid - 1);
        strncpy((char *)wc.sta.password, st.password, sizeof wc.sta.password - 1);
        wc.sta.threshold.authmode = st.password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        e = esp_wifi_set_config(WIFI_IF_STA, &wc);
        if (e != ESP_OK) ESP_LOGE(TAG, "STA config: %s", esp_err_to_name(e));
    }
    if (s_mode != 0) {
        memset(&wc, 0, sizeof wc);
        snprintf((char *)wc.ap.ssid, sizeof wc.ap.ssid, "%s", AP_SSID);
        snprintf((char *)wc.ap.password, sizeof wc.ap.password, "%s", AP_PASSWORD);
        wc.ap.ssid_len = strlen(AP_SSID);
        wc.ap.channel = 1;
        wc.ap.max_connection = 2;
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
        e = esp_wifi_set_config(WIFI_IF_AP, &wc);
        if (e != ESP_OK) ESP_LOGE(TAG, "AP config: %s", esp_err_to_name(e));
    }
    e = esp_wifi_start();
    s_started = e == ESP_OK;
    ESP_LOGI(TAG, "mode %s start: %s", wifi_net_mode_name(), esp_err_to_name(e));
    if (s_mode != 0) ESP_LOGI(TAG, "AP %s / %s - http://%s/", AP_SSID, AP_PASSWORD, s_ap_ip);
}


static void sntp_cb(struct timeval *tv)
{
    s_time_ok = true;
    ESP_LOGI(TAG, "time synced: %s", ctime(&tv->tv_sec));
}

static void net_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_mode != 1) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_time_ok = false;
        xEventGroupClearBits(s_ev, EV_GOT_IP);
        snprintf(s_ip, sizeof s_ip, "0.0.0.0");
        ESP_LOGW(TAG, "disconnected, retrying…");
        if (s_mode != 1) esp_wifi_connect();
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
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &net_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &net_evt, NULL));

    load_mode();
    configure_wifi();
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

void wifi_net_apply_settings(void)
{
    configure_wifi();
}

uint8_t wifi_net_mode(void) { return s_mode; }
const char *wifi_net_mode_name(void)
{
    return s_mode == 1 ? "AP" : s_mode == 2 ? "AP+STA" : "STA";
}
const char *wifi_net_ap_ip(void) { return s_mode == 0 ? "" : s_ap_ip; }
const char *wifi_net_ap_ssid(void) { return AP_SSID; }
const char *wifi_net_ap_password(void) { return AP_PASSWORD; }
bool wifi_net_set_mode(uint8_t mode)
{
    if (mode > 2) return false;
    nvs_handle_t h;
    if (nvs_open("station", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_u8(h, "wifi_mode", mode);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) return false;
    s_mode = mode;
    configure_wifi();
    return true;
}
