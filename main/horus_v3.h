#pragma once
#include <stdbool.h>
#include "horus_decoder.h"
bool horus_v3_parse(const char *hex, horus_v1_telemetry_t *out);
