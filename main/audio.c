/*
 * audio.c — ES8388 speaker output via esp_codec_dev.
 *
 * audio_play() (called from the demod task) interleaves mono→stereo and pushes
 * into a StreamBuffer; audio_out_task drains it into esp_codec_dev_write(), which
 * blocks on the I2S DMA — keeping that blocking call off the real-time demod path.
 */
#include "audio.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h" /* MALLOC_CAP_SPIRAM for the stream buffer */
#endif

#include "bsp_tab5.h"
#include "esp_timer.h"

static const char *TAG = "audio";

#define SB_BYTES (96 * 1024) /* buffer capacity; playback backlog is capped below */
#define AUDIO_MAX_BACKLOG_BYTES (32 * 1024) /* ~171 ms at 48k stereo 16-bit */

static esp_codec_dev_handle_t s_codec;
static StreamBufferHandle_t s_sb;
/* Touch tone request: UI only posts a flag; the audio task owns synthesis. */
static volatile uint32_t s_beep_request;
/* 0 = RX, 1 = TX monitor. Atomic access from Si5351 and audio tasks. */
static volatile uint32_t s_ft8_tx_active;
static volatile uint32_t s_ft8_tx_symbol;
static volatile uint32_t s_ft8_tx_is_ft4;
static volatile uint32_t s_ft8_tx_volume = 25u;

void audio_ft8_tx_monitor_set_volume(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    __atomic_store_n(&s_ft8_tx_volume, (uint32_t)percent, __ATOMIC_RELAXED);
}

int audio_ft8_tx_monitor_get_volume(void)
{
    return (int)__atomic_load_n(&s_ft8_tx_volume, __ATOMIC_RELAXED);
}

void audio_ft8_tx_monitor_set_ft4(bool ft4)
{
    __atomic_store_n(&s_ft8_tx_is_ft4, ft4 ? 1u : 0u, __ATOMIC_RELAXED);
}

void audio_ft8_tx_monitor_start(void)
{
    __atomic_store_n(&s_ft8_tx_symbol, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&s_ft8_tx_active, 1u, __ATOMIC_RELEASE);
}

void audio_ft8_tx_monitor_symbol(uint8_t symbol)
{
    if (symbol < 8u)
        __atomic_store_n(&s_ft8_tx_symbol, (uint32_t)symbol, __ATOMIC_RELAXED);
}

void audio_ft8_tx_monitor_stop(void)
{
    __atomic_store_n(&s_ft8_tx_active, 0u, __ATOMIC_RELEASE);
}


void audio_touch_beep(void)
{
    __atomic_store_n(&s_beep_request, 1u, __ATOMIC_RELAXED);
}


static void audio_out_task(void *arg)
{
    (void)arg;
    static int16_t buf[4096 / sizeof(int16_t)];
    uint32_t beep_left = 0;
    uint32_t phase = 0;
    uint32_t tx_phase = 0;
    bool was_tx = false;
    uint32_t tx_fade = 0;
    const uint32_t tx_fade_len = 240; /* 5 ms fade-in */
    /* 1.8 kHz square wave at 48 kHz, ~28 ms; low digital amplitude. */
    const uint32_t phase_step = (uint32_t)((1800ULL << 32) / AUDIO_SAMPLE_RATE);
    const uint32_t beep_samples = AUDIO_SAMPLE_RATE * 28 / 1000;
    while (1) {
        bool tx = __atomic_load_n(&s_ft8_tx_active, __ATOMIC_ACQUIRE) != 0;
        if (tx != was_tx) {
            /* Only the stream buffer consumer resets it. DSP is suppressed in TX. */
            xStreamBufferReset(s_sb);
            was_tx = tx;
            tx_fade = 0;
            tx_phase = 0;
            beep_left = 0;
        }
        if (tx) {
            /* 10 ms PCM frames; no extra producer in the RX stream buffer. */
            const size_t frames = 480;
            const uint32_t symbol = __atomic_load_n(&s_ft8_tx_symbol, __ATOMIC_RELAXED);
            /* Frequency: 1500 + symbol * 6.25 Hz. Phase accumulator, no libm. */
            const bool ft4 = __atomic_load_n(&s_ft8_tx_is_ft4, __ATOMIC_RELAXED) != 0;
            const uint64_t hz_x100 = 150000ULL + (uint64_t)symbol * (ft4 ? 2083ULL : 625ULL);
            const uint32_t step = (uint32_t)((hz_x100 * (1ULL << 32)) /
                                              ((uint64_t)AUDIO_SAMPLE_RATE * 100ULL));
            for (size_t i = 0; i < frames; ++i) {
                /* Low-amplitude triangle tone; smooth and cheap to synthesize. */
                uint32_t x = tx_phase >> 16;
                int32_t tri = (x < 32768u) ? (int32_t)x : (int32_t)(65535u - x);
                /* Scale the same monitor waveform by the independent TX slider.
                 * 100% = +/-16384 (50% of PCM full scale), 25% = +/-4096. */
                uint32_t vol = __atomic_load_n(&s_ft8_tx_volume, __ATOMIC_RELAXED);
                int32_t sample = (tri - 16384) * (int32_t)vol / 100;
                if (tx_fade < tx_fade_len) {
                    sample = sample * (int32_t)tx_fade / (int32_t)tx_fade_len;
                    ++tx_fade;
                }
                buf[2*i] = (int16_t)sample;
                buf[2*i+1] = (int16_t)sample;
                tx_phase += step;
            }
            (void)esp_codec_dev_write(s_codec, buf, frames * 4);
            continue;
        }
        size_t bytes = xStreamBufferReceive(s_sb, buf, sizeof buf, pdMS_TO_TICKS(10));
        if (__atomic_load_n(&s_ft8_tx_active, __ATOMIC_ACQUIRE)) continue;
        if (__atomic_exchange_n(&s_beep_request, 0u, __ATOMIC_RELAXED)) {
            beep_left = beep_samples;
            phase = 0;
        }
        if (!bytes && !beep_left) continue;
        if (!bytes) {
            /* Play even if demodulator is silent. Never enqueue into its
             * single-producer StreamBuffer from the UI task. */
            size_t frames = beep_left < 480 ? beep_left : 480;
            bytes = frames * 2 * sizeof(int16_t);
            memset(buf, 0, bytes);
        }
        size_t frames = bytes / (2 * sizeof(int16_t));
        for (size_t i = 0; i < frames && beep_left; ++i) {
            /* Fade edges to reduce clicks. */
            uint32_t elapsed = beep_samples - beep_left;
            int amp = 12000;
            if (elapsed < 96) amp = amp * (int)elapsed / 96;
            if (beep_left < 96) amp = amp * (int)beep_left / 96;
            int tone = (phase & 0x80000000u) ? amp : -amp;
            phase += phase_step;
            for (int ch = 0; ch < 2; ++ch) {
                int idx = (int)(2 * i + ch);
                int mixed = (int)buf[idx] + tone;
                if (mixed > 32767) mixed = 32767;
                if (mixed < -32768) mixed = -32768;
                buf[idx] = (int16_t)mixed;
            }
            --beep_left;
        }
        esp_codec_dev_write(s_codec, buf, bytes);
    }
}

void audio_init(void)
{
    s_codec = bsp_tab5_audio_speaker_init();
    if (!s_codec) {
        ESP_LOGE(TAG, "speaker codec init failed");
        return;
    }
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = AUDIO_SAMPLE_RATE,
    };
    if (esp_codec_dev_open(s_codec, &fs) != 0) {
        ESP_LOGE(TAG, "codec open failed");
        return;
    }
    esp_codec_dev_set_out_vol(s_codec, AUDIO_VOL_DEFAULT);

#ifdef ESP_PLATFORM
    /* 96 KB stream buffer in PSRAM (CPU-only data): the internal SRAM is
     * reserved for DMA users (hosted SDIO WiFi, USB) and is on the edge. */
    s_sb = xStreamBufferCreateWithCaps(SB_BYTES, 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!s_sb) /* PSRAM exhausted: fall back to internal RAM */
#endif
        s_sb = xStreamBufferCreate(SB_BYTES, 1);
    if (!s_sb) {
        ESP_LOGE(TAG, "audio stream buffer allocation failed");
        return;
    }
    xTaskCreatePinnedToCore(audio_out_task, "audio_out", 4096, NULL, 13, NULL, 1);
    ESP_LOGI(TAG, "speaker up at %d Hz", AUDIO_SAMPLE_RATE);
}

void audio_play(const int16_t *mono, int n)
{
    if (!s_sb || __atomic_load_n(&s_ft8_tx_active, __ATOMIC_ACQUIRE)) return;
    static int16_t st[512 * 2];
    int off = 0;
    while (off < n) {
        if (__atomic_load_n(&s_ft8_tx_active, __ATOMIC_ACQUIRE)) return;
        int chunk = n - off;
        if (chunk > 512) chunk = 512;
        for (int i = 0; i < chunk; i++) {
            int16_t s = mono[off + i];
            st[2 * i] = s;
            st[2 * i + 1] = s;
        }
        /* Only enqueue complete stereo frames. FreeRTOS StreamBufferSend can
         * otherwise accept a PARTIAL number of bytes when almost full,
         * destroying 16-bit stereo frame alignment for all later modes.
         *
         * Drop entire chunks if the audio task is falling behind, rather than
         * accumulating stale audio or ever writing a partial PCM frame.
         * Single producer (DSP) + single consumer (audio_out_task) assumed.
         */
        const size_t bytes = (size_t)chunk * 2u * sizeof(int16_t);
        const size_t used = xStreamBufferBytesAvailable(s_sb);
        if (used + bytes <= AUDIO_MAX_BACKLOG_BYTES &&
            xStreamBufferSpacesAvailable(s_sb) >= bytes) {
            (void)xStreamBufferSend(s_sb, st, bytes, 0);
        }
        off += chunk;
    }
}

void audio_set_volume(int vol)
{
    if (s_codec) esp_codec_dev_set_out_vol(s_codec, vol);
}
