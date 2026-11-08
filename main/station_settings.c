#include "station_settings.h"
#include "net_cfg.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>

void station_settings_get(station_settings_t *out)
{
    *out = (station_settings_t){0};
    snprintf(out->ssid, sizeof out->ssid, "%s", NET_WIFI_SSID);
    snprintf(out->password, sizeof out->password, "%s", NET_WIFI_PASSWORD);
    snprintf(out->callsign, sizeof out->callsign, "%s", PSK_CALLSIGN);
    snprintf(out->grid, sizeof out->grid, "%s", PSK_GRID);
    nvs_handle_t h;
    if (nvs_open("station", NVS_READONLY, &h) != ESP_OK) return;
    size_t n;
    n=sizeof out->ssid; nvs_get_str(h,"ssid",out->ssid,&n);
    n=sizeof out->password; nvs_get_str(h,"pass",out->password,&n);
    n=sizeof out->callsign; nvs_get_str(h,"call",out->callsign,&n);
    n=sizeof out->grid; nvs_get_str(h,"grid",out->grid,&n);
    nvs_close(h);
}
bool station_settings_save(const station_settings_t *in)
{
    if (!in || !in->ssid[0] || !in->callsign[0] || !in->grid[0]) return false;
    nvs_handle_t h;
    if (nvs_open("station", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_str(h,"ssid",in->ssid);
    if (e == ESP_OK) e = nvs_set_str(h,"pass",in->password);
    if (e == ESP_OK) e = nvs_set_str(h,"call",in->callsign);
    if (e == ESP_OK) e = nvs_set_str(h,"grid",in->grid);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}
