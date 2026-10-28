/*
 * psk.h — PSK Reporter upload (WSJT-X-style IPFIX datagrams over UDP/4739).
 *
 * Call psk_spot_msg() from every decoder when a message is decoded; it parses
 * callsign/locator from the decoded text, deduplicates, queues and a
 * background task sends batched datagrams when WiFi + SNTP time are up.
 */
#ifndef PSK_H
#define PSK_H

#include <stdbool.h>
#include <stdint.h>

void psk_init(void);

/* Queue one decoded spot for upload. `text` is the decoded message
 * ("CQ 9A1A JN85", "9A1A S52ABC R-05" for FT8; "K1ABC FN42 37" for WSPR).
 * `freq_hz` is the RF frequency of the signal (dial + audio offset),
 * `snr_db` the reported SNR. Drops silently when disabled/offline. */
void psk_spot_msg(const char *text, double freq_hz, int snr_db, const char *mode);

bool psk_enabled(void);
void psk_set_enabled(bool on); /* persists to NVS */

/* 0 = disabled, 1 = enabled but not (yet) online, 2 = enabled + WiFi + time */
int psk_status(void);

uint32_t psk_spots_sent(void);

#endif /* PSK_H */
