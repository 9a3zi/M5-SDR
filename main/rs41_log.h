#pragma once
#include <stdbool.h>
/* Mount the Tab5 internal microSD slot and start asynchronous RS41 CSV logging. */
bool rs41_log_start(void);
bool rs41_log_ready(void);
