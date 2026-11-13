#include "si5351.h"
#include "audio.h"
#include "bsp_tab5.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <time.h>
#include "esp_sntp.h"
#include "ft8/constants.h"
#include "ft8/message.h"
#include "ft8/encode.h"

static const char *TAG = "SI5351";

#define SI5351_I2C_ADDR 0x60
#define FT8_TX_RELAY_GPIO GPIO_NUM_47
#define FT8_RELAY_SETTLE_MS 30
#define FT8_RELAY_RELEASE_MS 30
#define SI5351_XTAL_HZ 25000000ULL

static int32_t calibration_ppm_x10 = 170;
static i2c_master_dev_handle_t s_dev;
static bool s_relay_ready;
static bool s_relay_tx;

/* LOW = RX (default), HIGH = TX. Hardware pull-down is required. */
static esp_err_t relay_set_tx(bool tx)
{
    if (!s_relay_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t err = gpio_set_level(FT8_TX_RELAY_GPIO, tx ? 1 : 0);
    if (err == ESP_OK) s_relay_tx = tx;
    return err;
}

bool si5351_relay_is_tx(void) { return s_relay_tx; }


int32_t si5351_get_calibration(void)
{
    return calibration_ppm_x10;
}

esp_err_t si5351_set_calibration(int32_t ppm_x10)
{
    if (ppm_x10 < -1000 || ppm_x10 > 1000)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("si5351", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(nvs, "cal_ppm10", ppm_x10);
    if (err == ESP_OK)
        err = nvs_commit(nvs);

    nvs_close(nvs);

    if (err == ESP_OK)
        calibration_ppm_x10 = ppm_x10;

    return err;
}

esp_err_t si5351_init(void)
{
    /* Force RX before attempting I2C initialization. */
    esp_err_t gpio_err = gpio_set_level(FT8_TX_RELAY_GPIO, 0);
    if (gpio_err != ESP_OK) return gpio_err;
    gpio_config_t relay_cfg = {
        .pin_bit_mask = (1ULL << FT8_TX_RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_err = gpio_config(&relay_cfg);
    if (gpio_err != ESP_OK) return gpio_err;
    s_relay_ready = true;
    s_relay_tx = false;

    i2c_master_bus_handle_t bus = bsp_tab5_get_i2c_bus();

    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_probe(
        bus,
        SI5351_I2C_ADDR,
        100
    );

if (err == ESP_OK) {
    nvs_handle_t nvs;

    if (nvs_open("si5351", NVS_READONLY, &nvs) == ESP_OK) {
        int32_t saved;

        if (nvs_get_i32(nvs, "cal_ppm10", &saved) == ESP_OK &&
            saved >= -1000 && saved <= 1000) {
            calibration_ppm_x10 = saved;
        }

        nvs_close(nvs);
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI5351_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
    if (err != ESP_OK) return err;
    /* Mute output at startup; never rely on Si5351 power-on defaults. */
    err = i2c_master_transmit(s_dev, (const uint8_t[]){3, 0xFF}, 2, 100);
    if (err != ESP_OK) return err;
    ESP_LOGW(TAG, "Si5351 detected - calibration %.1f ppm",
             calibration_ppm_x10 / 10.0);
} else {
        ESP_LOGW(TAG, "Si5351 not detected at 0x60");
    }

    return err;
}
// Test CLK0: 10 MHz
// Pretpostavka: Si5351 kristal 25 MHz

/* FT8 experimental TX. PLLA fractional tuning; phase continuity is NOT guaranteed. */
#define FT8_BASE_CHZ 1000000000ULL
#define SI5351_PLL_MIN_HZ 600000000ULL
#define SI5351_PLL_MAX_HZ 850000000ULL
#define PLL_DENOM 1000000U

static char s_tx_to[16] = "CQ";
static char s_tx_from[16] = "9A3ZI";
static char s_tx_extra[16] = "JN86";
static char s_tx_label[64] = "CQ 9A3ZI JN86";
/* Replace CQ symbols at a symbol boundary without dropping the TX relay. */
static volatile bool s_tone_running;
static volatile bool s_tone_stop;
static portMUX_TYPE s_swap_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_swap_tones[FT8_NN];
static volatile bool s_swap_pending;

esp_err_t si5351_ft8_replace_active_message(const char *to, const char *from, const char *extra)
{
    if (!s_tone_running || s_tone_stop || !to || !from || !extra)
        return ESP_ERR_INVALID_STATE;
    ftx_message_t msg;
    ftx_message_init(&msg);
    if (ftx_message_encode_std(&msg, NULL, to, from, extra) != FTX_MESSAGE_RC_OK)
        return ESP_ERR_INVALID_ARG;
    uint8_t new_tones[FT8_NN];
    ft8_encode(msg.payload, new_tones);
    portENTER_CRITICAL(&s_swap_mux);
    memcpy(s_swap_tones, new_tones, sizeof(new_tones));
    s_swap_pending = true;
    portEXIT_CRITICAL(&s_swap_mux);
    ESP_LOGW(TAG, "CQ replaced with directed FT8 reply at next symbol");
    return ESP_OK;
}

static volatile bool s_force_immediate_tx = false;
static volatile bool s_manual_takeover = false;
static int s_requested_tx_parity = -1; /* -1 = manual next slot */
#define FT8_FAST_LATE_MAX_US 1000000LL /* maximum allowed start delay */
#define FT8_FAST_PREP_US 80000LL
/* TX timing relative to the UTC 15-second boundary. Tune in 100 ms steps. */
#define FT8_TX_TIME_OFFSET_US 500000LL
static volatile int s_tone_index = -1;
static bool s_output_configured;
static uint32_t s_ms_div = 0;
/* Default preserves the previously tested 10 MHz output. */
static uint64_t s_ft8_base_chz = FT8_BASE_CHZ;

/* Selection is locked while a frame is queued or transmitting. */
esp_err_t si5351_ft8_set_base_cHz(uint64_t base_chz)
{
    if (s_tone_running) return ESP_ERR_INVALID_STATE;
    if (base_chz < 180000000ULL || base_chz > 3000000000ULL)
        return ESP_ERR_INVALID_ARG;
    s_ft8_base_chz = base_chz;
    return ESP_OK;
}
uint64_t si5351_ft8_get_base_cHz(void) { return s_ft8_base_chz; }

esp_err_t si5351_ft8_set_message(const char *to, const char *from, const char *extra)
{
    if (!to || !from || !extra || !*to || !*from || !*extra) return ESP_ERR_INVALID_ARG;
    if (strlen(to) >= sizeof s_tx_to || strlen(from) >= sizeof s_tx_from ||
        strlen(extra) >= sizeof s_tx_extra) return ESP_ERR_INVALID_ARG;
    if (s_tone_running) return ESP_ERR_INVALID_STATE;
    /* Validate with the actual FT8 encoder before accepting a new message. */
    ftx_message_t m;
    ftx_message_init(&m);
    if (ftx_message_encode_std(&m, NULL, to, from, extra) != FTX_MESSAGE_RC_OK)
        return ESP_ERR_INVALID_ARG;
    snprintf(s_tx_to, sizeof s_tx_to, "%s", to);
    snprintf(s_tx_from, sizeof s_tx_from, "%s", from);
    snprintf(s_tx_extra, sizeof s_tx_extra, "%s", extra);
    snprintf(s_tx_label, sizeof s_tx_label, "%s %s %s", to, from, extra);
    return ESP_OK;
}
const char *si5351_ft8_get_message(void) { return s_tx_label; }

static esp_err_t write_one(uint8_t reg, uint8_t val);

/* Register 3: bit 0 = 1 disables CLK0. */
static esp_err_t si5351_clk0_enable(bool enable)
{
    return write_one(3, enable ? 0xFE : 0xFF);
}


bool si5351_tone_test_running(void) { return s_tone_running; }
int si5351_tone_test_index(void) { return s_tone_index; }

static esp_err_t write_block(uint8_t first_reg, const uint8_t *bytes, size_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t data[9];
    if (len > 8) return ESP_ERR_INVALID_ARG;
    data[0] = first_reg;
    for (size_t i = 0; i < len; ++i) data[i + 1] = bytes[i];
    return i2c_master_transmit(s_dev, data, len + 1, 100);
}

static esp_err_t write_one(uint8_t reg, uint8_t val)
{
    return write_block(reg, &val, 1);
}

/* Set CLK0 with an even integer multisynth divider and fractional PLLA.
 * Keep divider constant throughout each FT8 frame (all tones span < 44 Hz).
 * No PLL reset between symbols. */
esp_err_t si5351_set_frequency_cHz(uint64_t freq_cHz)
{
    if (freq_cHz < 180000000ULL || freq_cHz > 3000000000ULL)
        return ESP_ERR_INVALID_ARG;
    if (!s_dev) return ESP_ERR_INVALID_STATE;

    /* Select the largest even divider that keeps PLLA within 600..850 MHz.
     * Divider range for multisynth 0 is 8..900; even integer dividers are
     * used to avoid divider changes between nearby FT8 symbols. */
    uint64_t freq_hz = freq_cHz / 100ULL;
    uint32_t div = (uint32_t)(SI5351_PLL_MAX_HZ / freq_hz);
    if (div > 900U) div = 900U;
    div &= ~1U;
    while (div >= 8U && freq_cHz * div > SI5351_PLL_MAX_HZ * 100ULL) div -= 2U;
    if (div < 8U || freq_cHz * div < SI5351_PLL_MIN_HZ * 100ULL)
        return ESP_ERR_INVALID_ARG;

    /* PLL multiplier in units of 1/PLL_DENOM, using centihertz input. */
    uint64_t scaled = (freq_cHz * div * PLL_DENOM + 1250000000ULL) / 2500000000ULL;
    int64_t corr = ((int64_t)scaled * calibration_ppm_x10 +
                    (calibration_ppm_x10 >= 0 ? 5000000LL : -5000000LL)) / 10000000LL;
    int64_t mult = (int64_t)scaled + corr;
    if (mult < 15LL * PLL_DENOM || mult > 90LL * PLL_DENOM)
        return ESP_ERR_INVALID_ARG;
    uint32_t a = (uint32_t)(mult / PLL_DENOM);
    uint32_t b = (uint32_t)(mult % PLL_DENOM);
    uint32_t f = (uint32_t)((128ULL * b) / PLL_DENOM);
    uint32_t p1 = 128U * a + f - 512U;
    uint32_t p2 = 128U * b - PLL_DENOM * f;
    uint32_t p3 = PLL_DENOM;
    uint8_t pll[8] = {
        (uint8_t)((p3 >> 8) & 0xff), (uint8_t)(p3 & 0xff),
        (uint8_t)((p1 >> 16) & 0x03), (uint8_t)(p1 >> 8), (uint8_t)p1,
        (uint8_t)(((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f)),
        (uint8_t)(p2 >> 8), (uint8_t)p2
    };
    esp_err_t err = write_block(26, pll, sizeof pll);
    if (err != ESP_OK) return err;

    if (!s_output_configured || s_ms_div != div) {
        /* Integer MS0: P1 = 128*divider - 512, P2=0, P3=1. */
        uint32_t mp1 = 128U * div - 512U;
        const uint8_t ms[8] = {
            0, 1, (uint8_t)((mp1 >> 16) & 0x03),
            (uint8_t)(mp1 >> 8), (uint8_t)mp1, 0, 0, 0
        };
        err = write_block(42, ms, sizeof ms);
        if (err != ESP_OK) return err;
        err = write_one(16, 0x4f); /* CLK0 8 mA, PLLA, integer MS */
        if (err != ESP_OK) return err;
        s_ms_div = div;
        s_output_configured = true;
        ESP_LOGI(TAG, "CLK0 divider /%lu", (unsigned long)div);
    }
    return ESP_OK;
}

esp_err_t si5351_test_10mhz(void)
{
    if (s_tone_running) return ESP_ERR_INVALID_STATE;
    esp_err_t err = si5351_clk0_enable(false);
    if (err == ESP_OK) err = si5351_set_frequency_cHz(FT8_BASE_CHZ);
    if (err == ESP_OK) err = relay_set_tx(true);
    if (err == ESP_OK) vTaskDelay(pdMS_TO_TICKS(FT8_RELAY_SETTLE_MS));
    if (err == ESP_OK) err = si5351_clk0_enable(true);
    if (err != ESP_OK) {
        /* Keep antenna on TX if muting the oscillator cannot be confirmed. */
        if (si5351_clk0_enable(false) == ESP_OK) (void)relay_set_tx(false);
    }
    if (err == ESP_OK) ESP_LOGI(TAG, "CLK0 10.000000 MHz, cal %.1f ppm", calibration_ppm_x10 / 10.0);
    else ESP_LOGE(TAG, "CLK0 10 MHz failed: %s", esp_err_to_name(err));
    return err;
}

static void wait_until_us(int64_t target)
{
    while (!s_tone_stop) {
        int64_t remain = target - esp_timer_get_time();
        if (remain <= 0) break;
        if (remain > 3000) {
            TickType_t ticks = pdMS_TO_TICKS((uint32_t)((remain - 1000) / 1000));
            vTaskDelay(ticks > 0 ? ticks : 1);
        } else {
            taskYIELD();
        }
    }
}

/* Cache a UTC-to-monotonic offset after the first valid clock read.
 * If another firmware module temporarily resets wall time, later STARTs
 * can still use the continuously running esp_timer clock. */
static bool s_utc_offset_valid = false;
static int64_t s_utc_minus_mono_us = 0;
static bool s_sntp_attempted = false;

static bool wait_for_ft8_slot(int64_t *start_us)
{
    ESP_LOGI(TAG, "WAIT: UTC clock and next 15-second slot");
    const int64_t deadline = esp_timer_get_time() + 60000000LL;
    int64_t last_log = 0;

    while (!s_tone_stop) {
        const int64_t mono = esp_timer_get_time();
        struct timeval tv;
        gettimeofday(&tv, NULL);
        const bool clock_valid = (int64_t)tv.tv_sec >= 1760000000LL;

        if (clock_valid) {
            int64_t wall_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
            s_utc_minus_mono_us = wall_us - mono;
            s_utc_offset_valid = true;
        } else if (!s_utc_offset_valid && !s_sntp_attempted) {
            s_sntp_attempted = true;
            if (!esp_sntp_enabled()) {
                esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
                esp_sntp_setservername(0, "pool.ntp.org");
                esp_sntp_init();
                ESP_LOGI(TAG, "WAIT: SNTP initialized");
            } else {
                ESP_LOGI(TAG, "WAIT: SNTP already active");
            }
        }

        if (s_utc_offset_valid) {
            const int64_t utc_us = mono + s_utc_minus_mono_us;
            int64_t current_slot = (utc_us / 15000000LL) * 15000000LL;
            int64_t elapsed = utc_us - current_slot;
            bool fast = s_requested_tx_parity >= 0 &&
                        ((current_slot / 15000000LL) & 1LL) == s_requested_tx_parity &&
                        elapsed <= FT8_FAST_LATE_MAX_US - FT8_FAST_PREP_US;
            int64_t next_slot = fast ? current_slot :
                                current_slot + 15000000LL;
            if (!fast && next_slot - utc_us < 500000LL)
                next_slot += 15000000LL;
            if (s_requested_tx_parity >= 0 &&
                ((next_slot / 15000000LL) & 1LL) != s_requested_tx_parity)
                next_slot += 15000000LL;
            /* Use the SAME UTC reference for regular and fast QSO TX.
             * Fast TX still uses the current parity-matched slot. If queued
             * after the calibrated boundary, start as soon as preparation
             * allows; we cannot move a past boundary backwards in time. */
            int64_t target_utc_us = next_slot + FT8_TX_TIME_OFFSET_US;
            if (fast && target_utc_us < utc_us + FT8_FAST_PREP_US)
                target_utc_us = utc_us + FT8_FAST_PREP_US;
            ESP_LOGI(TAG, "WAIT: %s UTC slot %lld, offset %lld ms, scheduled +%lld ms (clock=%s)",
                     fast ? "FAST" : "NORMAL",
                     (long long)(next_slot / 1000000LL),
                     (long long)(FT8_TX_TIME_OFFSET_US / 1000LL),
                     (long long)((target_utc_us - next_slot) / 1000LL),
                     clock_valid ? "system" : "cached");
            *start_us = target_utc_us - s_utc_minus_mono_us;
            return true;
        }
        if (mono >= deadline) {
            ESP_LOGE(TAG, "WAIT: UTC unavailable after 60 seconds");
            return false;
        }
        if (mono - last_log >= 5000000LL) {
            ESP_LOGW(TAG, "WAIT: system time=%lld, waiting for NTP",
                     (long long)tv.tv_sec);
            last_log = mono;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

/* Override an AUTO SEQ frame that is only queued, never one already on air. */
esp_err_t si5351_ft8_takeover_queued_send(const char *to, const char *from, const char *extra)
{
    if (!s_tone_running || s_tone_stop || s_relay_tx || s_tone_index >= 0)
        return ESP_ERR_INVALID_STATE;
    if (!to || !from || !extra) return ESP_ERR_INVALID_ARG;
    ftx_message_t msg;
    ftx_message_init(&msg);
    if (ftx_message_encode_std(&msg, NULL, to, from, extra) != FTX_MESSAGE_RC_OK)
        return ESP_ERR_INVALID_ARG;
    uint8_t new_tones[FT8_NN];
    ft8_encode(msg.payload, new_tones);
    portENTER_CRITICAL(&s_swap_mux);
    memcpy(s_swap_tones, new_tones, sizeof new_tones);
    s_swap_pending = true;
    s_manual_takeover = true;
    portEXIT_CRITICAL(&s_swap_mux);
    ESP_LOGW(TAG, "Manual SEND takes over queued AUTO frame");
    return ESP_OK;
}

static void tone_task(void *arg)
{
    (void)arg;
    ftx_message_t msg;
    ftx_message_init(&msg);
    uint8_t tones[FT8_NN];
    ftx_message_rc_t rc = ftx_message_encode_std(&msg, NULL, s_tx_to, s_tx_from, s_tx_extra);
    if (rc != FTX_MESSAGE_RC_OK) {
        ESP_LOGE(TAG, "FT8 message encoding failed: %d", (int)rc);
        goto finish;
    }
    ft8_encode(msg.payload, tones);

    if (si5351_clk0_enable(false) != ESP_OK) goto finish;
restart_schedule:
    if (s_manual_takeover) {
        s_force_immediate_tx = true;
        s_manual_takeover = false;
    }
    int64_t start_us = 0;
    int first_symbol = 0;
    if (s_force_immediate_tx) {
        /* WSJT-X-like late SEND: retain the original UTC symbol timeline.
         * Skip symbols already elapsed instead of extending the frame. */
        struct timeval tv;
        gettimeofday(&tv, NULL);
        if ((int64_t)tv.tv_sec < 1760000000LL) {
            ESP_LOGE(TAG, "LATE SEND: valid UTC time required");
            goto finish;
        }
        int64_t mono = esp_timer_get_time();
        int64_t wall_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
        int64_t slot_us = (wall_us / 15000000LL) * 15000000LL;
        if (s_requested_tx_parity >= 0 &&
            ((slot_us / 15000000LL) & 1LL) != s_requested_tx_parity) {
            ESP_LOGW(TAG, "LATE SEND: not our TX parity slot");
            goto finish;
        }
        start_us = mono + slot_us + FT8_TX_TIME_OFFSET_US - wall_us;
        /* Allow I2C setup and antenna relay settling before the first symbol. */
        int64_t ready_us = mono + 100000LL;
        if (ready_us > start_us) {
            first_symbol = (int)((ready_us - start_us + 159999LL) / 160000LL);
        }
        if (first_symbol >= FT8_NN) {
            ESP_LOGW(TAG, "LATE SEND: FT8 frame already finished");
            goto finish;
        }
        ESP_LOGW(TAG, "LATE SEND: skip %d symbols, finish at original UTC frame end", first_symbol);
    } else if (!wait_for_ft8_slot(&start_us)) goto finish;
    if (s_manual_takeover) goto restart_schedule;
    ESP_LOGI(TAG, "WAIT: prepared FT8 frame %s", s_tx_label);

    const uint64_t tx_base_chz = s_ft8_base_chz;
    ESP_LOGI(TAG, "FT8 RF base: %llu cHz", (unsigned long long)tx_base_chz);
    /* A decoded CQ reply can arrive while the next CQ is queued. */
    portENTER_CRITICAL(&s_swap_mux);
    if (s_swap_pending) {
        memcpy(tones, s_swap_tones, sizeof(tones));
        s_swap_pending = false;
    }
    portEXIT_CRITICAL(&s_swap_mux);
    /* Preload the first symbol while CLK0 is muted. */
    if (si5351_set_frequency_cHz(tx_base_chz + (uint64_t)tones[first_symbol] * 625ULL) != ESP_OK) goto finish;
    /* Stay in RX throughout the waiting period; key only just before TX. */
    if (!s_force_immediate_tx) {
        while (!s_tone_stop && !s_manual_takeover &&
               esp_timer_get_time() < start_us - (int64_t)FT8_RELAY_SETTLE_MS * 1000LL)
            vTaskDelay(pdMS_TO_TICKS(10));
        if (s_manual_takeover) goto restart_schedule;
    } else {
        wait_until_us(start_us + (int64_t)first_symbol * 160000LL -
                      (int64_t)FT8_RELAY_SETTLE_MS * 1000LL);
    }
    if (s_tone_stop) goto finish;
    if (relay_set_tx(true) != ESP_OK) goto finish;
    /* Mute SDR as soon as the antenna is switched to TX. */
    audio_ft8_tx_monitor_start();
    vTaskDelay(pdMS_TO_TICKS(FT8_RELAY_SETTLE_MS));
    /* Allow limited scheduler/relay latency, but never transmit a badly late frame. */
    const int64_t relay_lateness_us = esp_timer_get_time() - start_us;
    if (!s_force_immediate_tx && relay_lateness_us >= 100000LL) {
        ESP_LOGE(TAG, "TX relay missed slot boundary by %lld ms; aborting frame",
                 (long long)(relay_lateness_us / 1000LL));
        goto finish;
    }
    wait_until_us(start_us + (int64_t)first_symbol * 160000LL);
    if (s_tone_stop) goto finish;
    s_tone_index = first_symbol;
    if (si5351_clk0_enable(true) != ESP_OK) goto finish;
    audio_ft8_tx_monitor_symbol(tones[first_symbol]);

    for (int n = first_symbol + 1; n < FT8_NN && !s_tone_stop; ++n) {
        wait_until_us(start_us + (int64_t)n * 160000LL);
        if (s_tone_stop) break;
        portENTER_CRITICAL(&s_swap_mux);
        if (s_swap_pending) {
            memcpy(tones, s_swap_tones, sizeof(tones));
            s_swap_pending = false;
        }
        portEXIT_CRITICAL(&s_swap_mux);
        s_tone_index = n;
        esp_err_t err = si5351_set_frequency_cHz(tx_base_chz + (uint64_t)tones[n] * 625ULL);
        if (err == ESP_OK) audio_ft8_tx_monitor_symbol(tones[n]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "symbol %d I2C failed: %s", n, esp_err_to_name(err));
            break;
        }
    }
    if (!s_tone_stop) wait_until_us(start_us + (int64_t)FT8_NN * 160000LL);
finish:
    s_tone_index = -1;
    esp_err_t off_err = si5351_clk0_enable(false);
    if (off_err != ESP_OK) {
        ESP_LOGE(TAG, "CLK0 disable failed: %s; relay held in TX", esp_err_to_name(off_err));
    } else {
        vTaskDelay(pdMS_TO_TICKS(FT8_RELAY_RELEASE_MS));
        if (relay_set_tx(false) != ESP_OK)
            ESP_LOGE(TAG, "Failed to restore RX relay");
    }
    audio_ft8_tx_monitor_stop();
    ESP_LOGI(TAG, "DONE: FT8 output disabled");
    portENTER_CRITICAL(&s_swap_mux);
    s_swap_pending = false;
    portEXIT_CRITICAL(&s_swap_mux);
    s_manual_takeover = false;
    s_tone_running = false;
    vTaskDelete(NULL);
}

esp_err_t si5351_tone_test_start_parity(int parity)
{
    if (parity < -1 || parity > 1) return ESP_ERR_INVALID_ARG;
    if (s_tone_running || !s_dev) return ESP_ERR_INVALID_STATE;
    s_swap_pending = false;
    s_manual_takeover = false;
    s_requested_tx_parity = parity;
    s_force_immediate_tx = false;
    s_tone_stop = false;
    s_tone_index = -1;
    s_tone_running = true;
    if (xTaskCreate(tone_task, "si5351_tones", 4096, NULL, 4, NULL) != pdPASS) {
        s_tone_running = false;
        s_tone_index = -1;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* Manual SEND: join the current parity-matched FT8 frame, skipping elapsed symbols. */
esp_err_t si5351_tone_test_start_immediate(void)
{
    if (s_tone_running || !s_dev) return ESP_ERR_INVALID_STATE;
    /* Use the already selected CQ parity for the late current-slot send. */
    s_force_immediate_tx = true;
    s_manual_takeover = false;
    s_tone_stop = false;
    s_tone_index = -1;
    s_tone_running = true;
    if (xTaskCreate(tone_task, "si5351_tones", 4096, NULL, 4, NULL) != pdPASS) {
        s_tone_running = false;
        s_force_immediate_tx = false;
        s_tone_index = -1;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t si5351_tone_test_start(void)
{
    return si5351_tone_test_start_parity(-1);
}

void si5351_tone_test_stop(void)
{
    ESP_LOGW(TAG, "CQ DIAG: stop requested running=%d relay=%d", (int)s_tone_running, (int)s_relay_tx);
    s_tone_stop = true;
    /* Also stop the continuous 10 MHz calibration carrier when idle. */
    if (!s_tone_running && s_dev) {
        if (si5351_clk0_enable(false) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(FT8_RELAY_RELEASE_MS));
            (void)relay_set_tx(false);
        }
    }
}

/* Apply calibration immediately without writing flash. */
esp_err_t si5351_apply_calibration(int32_t ppm_x10)
{
    if (ppm_x10 < -1000 || ppm_x10 > 1000) return ESP_ERR_INVALID_ARG;
    if (s_tone_running) return ESP_ERR_INVALID_STATE;
    int32_t previous = calibration_ppm_x10;
    calibration_ppm_x10 = ppm_x10;
    esp_err_t err = si5351_test_10mhz();
    if (err != ESP_OK) {
        calibration_ppm_x10 = previous;
        (void)si5351_test_10mhz();
    }
    return err;
}
