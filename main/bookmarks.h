/*
 * bookmarks.h — band presets for the HF / VHF / UHF buttons.
 *
 * Each button cycles through its list (a single entry = it just returns to the
 * band start). Add entries to a list to get more presets on that button.
 *
 *   mode    : DSP_MODE_* demodulator to select
 *   rate    : RTL2832 sample rate to use, 0 = keep the current one.
 *             1152000 = 1.15 MHz wide view. 288000 = HF mode: the RTL2832 cannot
 *             go down to ~100 kS/s (it only accepts 225001..300000 and
 *             900001..3200000), so it runs at 288 kS/s and dsp.c decimates to
 *             96 kS/s for the spectrum (96 kHz span, 94 Hz per bin).
 *   step_hz : tuning step (must be one of the STEPS[] values in ui.c).
 *
 * Labels use upper case only: the built-in font has no lower-case letters.
 */
#pragma once
#include <stdint.h>
#include "dsp.h"

typedef struct {
    const char *name;
    uint32_t hz;
    uint8_t mode;
    uint32_t rate;
    uint32_t step_hz;
} bookmark_t;

/* HF amateur bands, cycled with each press of the HF button. */
static const bookmark_t BM_HF[] = {
    {"160M 1.840",   1840000, DSP_MODE_LSB, 288000, 500},
    {"80M 3.650",    3650000, DSP_MODE_LSB, 288000, 500},
    {"60M 5.357",    5357000, DSP_MODE_USB, 288000, 500},
    {"40M 7.100",    7100000, DSP_MODE_LSB, 288000, 500},
    {"30M 10.120",  10120000, DSP_MODE_USB, 288000, 500},
    {"20M 14.200",  14200000, DSP_MODE_USB, 288000, 500},
    {"17M 18.100",  18100000, DSP_MODE_USB, 288000, 500},
    {"15M 21.200",  21200000, DSP_MODE_USB, 288000, 500},
    {"12M 24.930",  24930000, DSP_MODE_USB, 288000, 500},
    {"10M 28.500",  28500000, DSP_MODE_USB, 288000, 500},
    {"6M 50.150",   50150000, DSP_MODE_USB, 1152000, 500},
};

/* VHF: start at 144.000 MHz, narrowband FM. */
static const bookmark_t BM_VHF[] = {
    {"VHF 144 MHZ", 144000000, DSP_MODE_NFM, 1152000, 12500},
};

/* UHF: start at 430.000 MHz, narrowband FM. */
static const bookmark_t BM_UHF[] = {
    {"UHF 430 MHZ", 430000000, DSP_MODE_NFM, 1152000, 12500},
};

#define BM_HF_N  (int)(sizeof(BM_HF) / sizeof(BM_HF[0]))
#define BM_VHF_N (int)(sizeof(BM_VHF) / sizeof(BM_VHF[0]))
#define BM_UHF_N (int)(sizeof(BM_UHF) / sizeof(BM_UHF[0]))
