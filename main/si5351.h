#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t si5351_init(void);
esp_err_t si5351_test_10mhz(void);
int32_t si5351_get_calibration(void);
esp_err_t si5351_set_calibration(int32_t ppm_x10);
esp_err_t si5351_apply_calibration(int32_t ppm_x10);
esp_err_t si5351_set_frequency_cHz(uint64_t freq_cHz);
/* Select a standard FT8 message (TO FROM GRID/REPORT/ACK) while idle. */
esp_err_t si5351_ft8_set_message(const char *to, const char *from, const char *extra);
const char *si5351_ft8_get_message(void);
esp_err_t si5351_tone_test_start(void);
/* Change an ongoing CQ to a QSO reply without restarting the FT8 frame. */
esp_err_t si5351_ft8_replace_active_message(const char *to, const char *from, const char *extra);
/* TX only on UTC 15-second slots with specified parity (0 or 1). */
esp_err_t si5351_tone_test_start_parity(int parity);
/* Manual SEND: start late in the current UTC slot, skipping elapsed symbols. */
esp_err_t si5351_tone_test_start_immediate(void);
/* Manual SEND overrides a queued (not transmitting) AUTO frame. */
esp_err_t si5351_ft8_takeover_queued_send(const char *to, const char *from, const char *extra);
void si5351_tone_test_stop(void);
bool si5351_tone_test_running(void);
bool si5351_relay_is_tx(void);
esp_err_t si5351_ft8_set_base_cHz(uint64_t base_chz);
uint64_t si5351_ft8_get_base_cHz(void);
int si5351_tone_test_index(void);
/* FT4 transmit backend; FT4 parity is based on 7.5-second UTC slots. */
esp_err_t si5351_ft4_set_message(const char *to, const char *from, const char *extra);
esp_err_t si5351_ft4_start_parity(int parity);
#ifdef __cplusplus
}
#endif
