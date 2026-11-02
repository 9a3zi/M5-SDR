#pragma once
#include <stdint.h>
#include <stdbool.h>
void web_mirror_load_settings(void);
void web_mirror_start(void);
void web_mirror_set_enabled(bool enabled);
bool web_mirror_enabled(void);
void web_mirror_publish(const uint16_t *pixels);
bool web_mirror_pointer(int *x, int *y, bool *down);
