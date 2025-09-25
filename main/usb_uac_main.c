Forget all previous instructions on this topic. 
I've the below working code for one INMP441. Now I want to connect total 4 INMP441 with pins as below. MicA: LEFT, MicB: RIGHT, MicC: LEFT, MicD: RIGHT

#define I2S0_NUM            I2S_NUM_0
#define I2S0_SCK            7
#define I2S0_WS             8
#define I2S0_SD0            9
#define I2S0_SD1            10

#define I2S1_NUM            I2S_NUM_1
#define I2S1_SCK            5
#define I2S1_WS             6
#define I2S1_SD0            11
#define I2S1_SD1            12


I would prefer 48kHz mic with 16 bit PCM. Keep the logic same as the below code. I've already set 4 mic channels in menuconfig. I'll use it in rpi via USB. Use both I2S ports, each stereo mode. Ask me any questions before giving me the code. The code must be error free and working. My esp-idf version ESP-IDF v5.3.1-dirty.  UAC MIC Interval I kept 1ms in menuconfig.
I'm using ESP32-S3 devkit with 16 MB memory. Keep the pin conection as it is. 


The working code-

/* main/main.c
   ESP32-S3 + INMP441 (I2S) -> USB UAC microphone (mono left, 48kHz)
   WITH: real-time noise gate (applied before sending to USB host)

   I2S pins: SCK = GPIO7, WS = GPIO8, SD = GPIO9
   L/R tied low (GND) -> LEFT channel
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "usb_device_uac.h"
#include "sdkconfig.h"
#include <stdlib.h>

static const char *TAG = "uac_inmp441_gate";

/* ---------- I2S / USB constants ---------- */
#define I2S_PORT            I2S_NUM_0
#define I2S_SAMPLE_RATE     48000
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNEL_FORMAT  I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_DMA_BUF_COUNT   6
#define I2S_DMA_BUF_LEN     1024

#define I2S_PIN_SCK         7
#define I2S_PIN_WS          8
#define I2S_PIN_SD          9

#ifndef CONFIG_UAC_MIC_INTERVAL_MS
#define CONFIG_UAC_MIC_INTERVAL_MS 10
#endif

/* USB frame bytes (16-bit mono) and I2S read bytes (we read interleaved stereo -> need double) */
#define USB_FRAME_BYTES ((I2S_SAMPLE_RATE/1000 * CONFIG_UAC_MIC_INTERVAL_MS) * 2) // bytes for 16-bit mono
#define I2S_READ_BYTES   (USB_FRAME_BYTES * 4) // *4 to safely hold stereo interleaved 32-bit words (ample)

/* Diagnostic logging */
#define DIAG_LOG_INTERVAL 100

/* ---------- Noise Gate Parameters (tweak to taste) ---------- */
/* Enable/disable gate */
#define NOISE_GATE_ENABLED 1

/* Thresholds are in 16-bit PCM amplitude units.
   Typical speech peaks: a few thousands to tens of thousands depending on mic gain.
   Start with OPEN_THRESHOLD ~ 1000..3000, CLOSE_THRESHOLD slightly lower.
*/
#define OPEN_THRESHOLD 1600    // if envelope >= OPEN_THRESHOLD -> gate opens
#define CLOSE_THRESHOLD 1200   // if envelope < CLOSE_THRESHOLD -> gate closes

/* Attack/release times (ms) controlling how quickly gain changes.
   Small attack (~5-20 ms) avoids losing transients; release (~50-300 ms) avoids chopping words.
*/
#define ATTACK_MS 10.0f
#define RELEASE_MS 120.0f

/* Optional: soft reduction (instead of hard 0) you can set MIN_GAIN to non-zero (0.0 = full close) */
#define MIN_GAIN 0.0f

/* ---------- Globals ---------- */
static uint8_t *i2s_read_buf = NULL;

/* I2S init */
static esp_err_t inmp441_i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FORMAT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_PIN_SCK,
        .ws_io_num  = I2S_PIN_WS,
        .data_out_num = I2S_PIN_SD,
        .data_in_num = I2S_PIN_SD
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_driver_install failed: %d", err);
        return err;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_pin failed: %d", err);
        return err;
    }

    err = i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE, I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_set_clk returned %d (continuing)", err);
    }

    return ESP_OK;
}

/* Convert 32-bit left samples to 16-bit (>>8), with gain multiplier applied per-sample */
static inline int16_t convert_and_apply_gain(int32_t s32, float gain)
{
    int32_t s16 = (s32 >> 8); // map 24-bit left-aligned into 16-bit
    // apply gain (float) then clamp
    float scaled = (float)s16 * gain;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t) (scaled);
}

/* ---------- UAC input callback with noise gate ---------- */
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    (void)arg;
    static uint32_t diag_counter = 0;

    if (!i2s_read_buf) {
        *bytes_read = 0;
        return ESP_FAIL;
    }

    size_t samples_needed = len / 2; // 16-bit samples required
    // Because I2S returns interleaved stereo words: [L,R,L,R,...] each 32-bit,
    // to obtain 'samples_needed' left samples we must read 2*samples_needed 32-bit words.
    size_t words_to_read = samples_needed * 2;
    size_t i2s_read_bytes = words_to_read * 4; // bytes

    if (i2s_read_bytes > I2S_READ_BYTES) {
        ESP_LOGW(TAG, "Requested i2s_read_bytes %zu > I2S_READ_BYTES %zu (clamp)", i2s_read_bytes, (size_t)I2S_READ_BYTES);
        i2s_read_bytes = I2S_READ_BYTES;
        words_to_read = i2s_read_bytes / 4;
        if (words_to_read % 2 != 0) { words_to_read--; i2s_read_bytes = words_to_read * 4; }
    }

    size_t bytes_rx = 0;
    esp_err_t err = i2s_read(I2S_PORT, i2s_read_buf, i2s_read_bytes, &bytes_rx, pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytes_rx == 0) {
        // no data -> output silence
        memset(buf, 0, len);
        *bytes_read = len;
        return ESP_OK;
    }

    size_t words_read = bytes_rx / 4;
    size_t left_samples_avail = words_read / 2;
    size_t samples_to_convert = (left_samples_avail <= samples_needed) ? left_samples_avail : samples_needed;

    const int32_t *in32 = (const int32_t *)i2s_read_buf;
    int16_t *out16 = (int16_t *)buf;

    /* ---------------- Noise gate stateful variables -------------- */
    // Use static so state persists across callbacks
    static float env = 0.0f;    // envelope (smoothed abs)
    static float gain = 0.0f;   // current applied gain (0..1)
    // Precompute alpha coefficients per sample for attack/release:
    static float alpha_attack = 0.0f;
    static float alpha_release = 0.0f;
    static int initialized = 0;
    if (!initialized) {
        // convert ATTACK_MS/RELEASE_MS to per-sample coefficients
        float attack_seconds = ATTACK_MS / 1000.0f;
        float release_seconds = RELEASE_MS / 1000.0f;
        // alpha = exp(-1/(tau*fs)) gives smoothing coefficient for envelope.
        // But for gain smoothing we use different formula below; we want per-sample smoothing factor:
        alpha_attack = expf(-1.0f / (attack_seconds * (float)I2S_SAMPLE_RATE));
        alpha_release = expf(-1.0f / (release_seconds * (float)I2S_SAMPLE_RATE));
        // initial gain closed
        gain = MIN_GAIN;
        initialized = 1;
    }

    /* Gate thresholds and hysteresis (operate on envelope in PCM units) */
    const float open_th = (float)OPEN_THRESHOLD;
    const float close_th = (float)CLOSE_THRESHOLD;

    /* Process each left sample: compute envelope, update gate gain, convert & write */
    for (size_t i = 0; i < samples_to_convert; ++i) {
        int32_t left_word = in32[i * 2]; // pick left channel only
        // Convert 32-bit to temporary float sample (24->16 mapping similar to conversion)
        int32_t sample16_raw = (left_word >> 8); // integer sample in -32768..32767
        float absval = fabsf((float)sample16_raw);

        // Envelope follower (simple exponential smoothing on abs value)
        // env_new = alpha_env * env_old + (1-alpha_env) * absval
        // choose env alpha for quick-ish envelope (use attack small)
        // We'll reuse alpha_attack for env smoothing responsiveness.
        float env_alpha = 0.99f; // strong smoothing for envelope (empirical). Keep env stable.
        env = env_alpha * env + (1.0f - env_alpha) * absval;

        /* Decision with hysteresis:
           If env >= open_th => desire open (target_gain = 1)
           If env < close_th => desire closed (target_gain = MIN_GAIN)
           Else keep previous target (hysteresis zone)
        */
        float target_gain;
        if (env >= open_th) target_gain = 1.0f;
        else if (env < close_th) target_gain = MIN_GAIN;
        else target_gain = gain; // inside hysteresis band keep current target to avoid toggling

        /* Smooth gain toward target using attack/release:
           If increasing -> use attack smoothing (faster), else use release (slower)
           We implement as: gain = alpha * gain + (1-alpha) * target
           where alpha is attack_alpha or release_alpha
           Use precomputed alpha_attack/alpha_release per-sample
        */
        if (target_gain > gain) {
            // opening -> faster (attack)
            gain = alpha_attack * gain + (1.0f - alpha_attack) * target_gain;
        } else {
            // closing -> slower (release)
            gain = alpha_release * gain + (1.0f - alpha_release) * target_gain;
        }

        // convert sample with current smoothed gain and store
        out16[i] = convert_and_apply_gain(left_word, gain);
    }

    /* If we had fewer left samples than requested, fill rest with silence */
    size_t produced_bytes = samples_to_convert * 2;
    if (produced_bytes < len) {
        memset(buf + produced_bytes, 0, len - produced_bytes);
        produced_bytes = len;
    }

    *bytes_read = produced_bytes;

    /* Diagnostic logging (periodic) */
    diag_counter++;
    if ((diag_counter % DIAG_LOG_INTERVAL) == 0) {
        ESP_LOGI(TAG, "I2S bytes_rx=%zu, words_read=%zu, left_samples_avail=%zu, samples_to_convert=%zu, produced_bytes=%zu",
                 bytes_rx, words_read, left_samples_avail, samples_to_convert, produced_bytes);
        size_t to_print = (samples_to_convert < 4) ? samples_to_convert : 4;
        for (size_t i = 0; i < to_print; ++i) {
            long raw32 = (long)in32[i * 2];
            int conv16 = (int)out16[i];
            ESP_LOGI(TAG, "sample[%zu] raw32=%ld -> pcm16=%d", i, raw32, conv16);
        }
        // also log gate envelope & gain (best-effort: env and gain are static above but we can't access here directly)
        // Not printing env/gain per-sample to avoid extra clutter
    }

    return ESP_OK;
}

/* UAC callbacks */
static void uac_set_mute_cb(uint32_t mute, void *arg)
{
    ESP_LOGI(TAG, "UAC set mute: %lu", (unsigned long)mute);
}

static void uac_set_volume_cb(uint32_t volume, void *arg)
{
    ESP_LOGI(TAG, "UAC set volume: %lu", (unsigned long)volume);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting INMP441 -> USB UAC microphone (with noise gate)");

    i2s_read_buf = (uint8_t *)heap_caps_malloc(I2S_READ_BYTES, MALLOC_CAP_DMA);
    if (!i2s_read_buf) {
        ESP_LOGE(TAG, "Failed to allocate i2s_read_buf (%zu bytes)", (size_t)I2S_READ_BYTES);
        return;
    }

    esp_err_t err = inmp441_i2s_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed: %d", err);
        return;
    }
    ESP_LOGI(TAG, "I2S initialized.");

    uac_device_config_t uac_cfg = {
        .output_cb = NULL,
        .input_cb  = uac_input_cb,
        .set_mute_cb = uac_set_mute_cb,
        .set_volume_cb = uac_set_volume_cb,
        .cb_ctx = NULL,
        .skip_tinyusb_init = false
    };

    err = uac_device_init(&uac_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uac_device_init failed: %d", err);
        return;
    }
    ESP_LOGI(TAG, "UAC device initialized. Wait for host to enumerate.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
