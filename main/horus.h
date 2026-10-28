#pragma once
/* RTL-SDR 48 ksps 4FSK symbol diagnostics only; not a Horus packet decoder. */
void horus_reset(void);
void horus_feed(float phase_delta);
void horus_status(char *dst, unsigned capacity);
