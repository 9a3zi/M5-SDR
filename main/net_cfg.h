/*
 * net_cfg.h — network + PSK Reporter identity settings.
 *
 * EDIT THIS FILE before flashing: your WiFi credentials and station identity.
 * The PSK Reporter enable flag is stored in NVS and toggled from the UI
 * (the "PSK" checkbox); these defines are only the compile-time defaults
 * for the identity and WiFi credentials.
 */
#ifndef NET_CFG_H
#define NET_CFG_H

/* ---- WiFi (station) ------------------------------------------------------ */
#define NET_WIFI_SSID     "UR-SSID"
#define NET_WIFI_PASSWORD "UR-PSWD"

/* Timezone for the band-plan scheduler (POSIX TZ string; SNTP syncs UTC and
 * this converts it to local time for the schedule). Central European Time
 * with DST is the default — change to your zone, e.g. "UTC0", "EST5EDT", … */
#define NET_TZ            "CET-1CEST,M3.5.0,M10.5.0/3"

/* ---- PSK Reporter identity ------------------------------------------------ */
/* Your callsign as the receiving station (uploaded with every report). */
#define PSK_CALLSIGN      "CALLSIGN"
/* Your locator, 4 or 6 characters (e.g. "JN76", "FN42AB"). */
#define PSK_GRID          "LOCATOR"
/* Software descriptor shown on pskreporter.info. */
#define PSK_SW_NAME       "M5-SDR v1.3"
/* Receiver rig description (optional rigInformation field, 30351.13). */
#define PSK_RIG           "M5Stack Tab5 + RTL-SDR"

/* Upload destination. For debugging you can point this at a PC running a UDP
 * listener on port 4739 (e.g. python -c "import socket;s=socket.socket(2,2);
 * s.bind(('0.0.0.0',4739));print(s.recvfrom(2048))") to capture the datagram. */
#define PSK_HOST          "report.pskreporter.info"
/* PSK Reporter also runs a packet-analysis test listener on port 14739:
 * send there and inspect your datagram at
 * https://pskreporter.info/cgi-bin/packet-analysis.pl */
#define PSK_PORT          4739

/* Default state of the UI "PSK" checkbox on first boot (then NVS holds it). */
#define PSK_ENABLED_DEFAULT 1

/* Upload pacing: flush a datagram at the latest this many seconds after the
 * oldest queued spot even with few decodes (PSK Reporter asks for at most one
 * datagram per 5 min for pure timer flushes; spot-triggered flushes may go
 * sooner). */
#define PSK_FLUSH_MAX_AGE_S   150
#define PSK_FLUSH_MIN_SPOTS   6

#endif /* NET_CFG_H */
