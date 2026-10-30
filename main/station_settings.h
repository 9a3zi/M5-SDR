#pragma once
#include <stdbool.h>
#include <stddef.h>
#define ST_SSID_LEN 33
#define ST_PASS_LEN 65
#define ST_CALL_LEN 16
#define ST_GRID_LEN 9
typedef struct { char ssid[ST_SSID_LEN]; char password[ST_PASS_LEN]; char callsign[ST_CALL_LEN]; char grid[ST_GRID_LEN]; } station_settings_t;
void station_settings_get(station_settings_t *out);
bool station_settings_save(const station_settings_t *in);
