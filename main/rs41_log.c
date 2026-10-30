/* Tab5 microSD RS41 telemetry logger; ESP-IDF 5.4 SDMMC 4-bit interface. */
#include "rs41_log.h"
#include "rs41.h"
#include "sdr.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "rs41_sd";
static bool mounted;
static sdmmc_card_t *card;

bool rs41_log_ready(void) { return mounted; }

static void log_task(void *arg)
{
    (void)arg;
    uint32_t last_seq = 0;
    for (;;) {
        rs41_info_t r;
        if (rs41_get_info(&r) && r.seq != last_seq) {
            last_seq = r.seq;
            ESP_LOGI(TAG, "Frame seq=%lu serial=%s have_id=%d crc_blocks=%u",
                     (unsigned long)r.seq, r.id, (int)r.have_id, (unsigned)r.blk_ok);
            if (r.have_id && r.id[0] && r.blk_ok) {
                char serial[sizeof(r.id)];
                size_t n = strnlen(r.id, sizeof(r.id));
                if (n >= sizeof(serial)) n = sizeof(serial) - 1;
                for (size_t i = 0; i < n; i++) {
                    unsigned char c = (unsigned char)r.id[i];
                    serial[i] = (isalnum(c) || c == '-' || c == '_') ? c : '_';
                }
                serial[n] = 0;
                char path[80];
                snprintf(path, sizeof(path), "/sdcard/RS41/%s.csv", serial);
                struct stat st;
                bool fresh = stat(path, &st) != 0;
                FILE *f = fopen(path, "a");
                if (f) {
                    if (fresh) fputs("serial,frame,frequency_hz,gps_week,gps_tow_ms,utc_hh,utc_mm,utc_ss,has_position,latitude,longitude,altitude_m,speed_ms,heading_deg,climb_ms,satellites,rs_fixed,crc_blocks\n", f);
                    sdr_stats_t stats;
                    sdr_get_stats(&stats);
                    fprintf(f, "%s,%u,%lu,%u,%lu,%02d,%02d,%02d,%u,",
                            serial, (unsigned)r.frame_no, (unsigned long)stats.center_freq,
                            (unsigned)r.week, (unsigned long)r.tow_ms, r.hh, r.mm, r.ss,
                            r.have_pos ? 1u : 0u);
                    if (r.have_pos)
                        fprintf(f, "%.7f,%.7f,%.2f,%.2f,%.1f,%.2f,%u,",
                                r.lat, r.lon, r.alt, r.vh, r.vd, r.vv, (unsigned)r.sats);
                    else fputs(",,,,,,,", f);
                    fprintf(f, "%d,%u\n", r.rs_fixed, (unsigned)r.blk_ok);
                    if (fflush(f) != 0) ESP_LOGE(TAG, "Flush failed: %s", path);
                    if (fclose(f) != 0) ESP_LOGE(TAG, "Close failed: %s", path);
                    else ESP_LOGI(TAG, "CSV append: %s frame=%u", path, (unsigned)r.frame_no);
                } else ESP_LOGE(TAG, "Cannot append %s (errno=%d)", path, errno);
            } else {
                ESP_LOGW(TAG, "Frame not logged: missing serial or CRC-valid block");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool rs41_log_start(void)
{
    if (mounted) return true;
sdmmc_host_t host = SDMMC_HOST_DEFAULT();
host.slot = SDMMC_HOST_SLOT_0;
host.max_freq_khz = 400;

sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
slot.width = 4;
slot.clk = GPIO_NUM_43;
slot.cmd = GPIO_NUM_44;
slot.d0 = GPIO_NUM_39;
slot.d1 = GPIO_NUM_40;
slot.d2 = GPIO_NUM_41;
slot.d3 = GPIO_NUM_42;
slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s; RS41 receiver continues", esp_err_to_name(err));
        return false;
    }
    if (mkdir("/sdcard/RS41", 0775) != 0) {
        struct stat st;
        if (stat("/sdcard/RS41", &st) != 0 || !S_ISDIR(st.st_mode)) {
            ESP_LOGE(TAG, "Cannot create /sdcard/RS41");
            esp_vfs_fat_sdcard_unmount("/sdcard", card);
            card = NULL;
            return false;
        }
    }
    mounted = true;
    if (xTaskCreate(log_task, "rs41_sd_log", 4096, NULL, 3, NULL) != pdPASS) {
        mounted = false;
        esp_vfs_fat_sdcard_unmount("/sdcard", card);
        card = NULL;
        return false;
    }
    ESP_LOGI(TAG, "RS41 CSV logging ready at /sdcard/RS41");
    return true;
}
