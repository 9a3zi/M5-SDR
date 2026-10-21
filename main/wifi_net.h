/*
 * wifi_net.h — WiFi station + SNTP time for the P4 (ESP32-C6 radio via
 * esp_hosted / esp_wifi_remote — the esp_wifi API is the standard one).
 */
#ifndef WIFI_NET_H
#define WIFI_NET_H

#include <stdbool.h>
#include <stdint.h>

/* Starts the station: netif, events, connect with the net_cfg.h credentials,
 * and SNTP (pool.ntp.org) once an IP is obtained. Call once after ui_start().
 * Never fails fatally — without WiFi the receiver keeps working. */
void wifi_net_init(void);

bool wifi_net_connected(void); /* has an IP address */
bool wifi_net_time_ok(void);   /* SNTP has set the clock (needed for reports) */
const char *wifi_net_ip(void); /* "0.0.0.0" when not connected */

#endif /* WIFI_NET_H */
