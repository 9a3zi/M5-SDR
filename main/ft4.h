#pragma once
/* FT4 receiver uses the same message/status representation as FT8. */
#include "ft8.h"
#define FT4_TEXT_MAX FT8_TEXT_MAX
#ifdef __cplusplus
extern "C" {
#endif
void ft4_start(void);
void ft4_reset(void);
void ft4_set_qso_focus(const char *callsign, int audio_hz);
void ft4_feed(float y);
int ft4_get_messages(ft8_msg_t *out, int max, ft8_status_t *st);
void ft4_process(void);
#ifdef __cplusplus
}
#endif
