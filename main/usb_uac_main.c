// main.c - auto-detect I2S alignment for INMP441 and stream to USB UAC
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

// Replace this with your USB UAC header / init function prototype
// e.g. #include "usb_uac.h"
// Here we assume a uac_device_init() that accepts a config struct with input_cb
typedef struct {
    esp_err_t (*input_cb)(uint8_t *data, size_t max_len, size_t *bytes_written, void *arg);
    void *output_cb;
} uac_device_config_t;
extern esp_err_t uac_device_init(const uac_device_config_t *cfg);

static const char *TAG = "USB_MIC_AUTO";

#define PIN_I2S_BCLK    41
#define PIN_I2S_WS      42
#define PIN_I2S_DIN     2

#define IN_BUF_WORDS    512   // compile-time constant for buffer size (words)
#define BUFFER_WORDS    1024  // words for evaluation read
#define SAMPLE_RATE_TRIES 2

static i2s_chan_handle_t rx_chan = NULL;
static SemaphoreHandle_t dbg_lock = NULL;

/* Helper: init i2s rx channel with provided flags */
static esp_err_t i2s_init_std(i2s_chan_handle_t *chan,
                              bool left_align,
                              bool bit_shift,
                              int sample_rate_hz)
{
    if (*chan) {
        i2s_channel_disable(*chan);
        i2s_del_channel(*chan);
        *chan = NULL;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, chan);
    if (err != ESP_OK) return err;

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = sample_rate_hz,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_LEFT,
            .ws_width       = I2S_SLOT_BIT_WIDTH_32BIT,
            .ws_pol         = false,
            .bit_shift      = bit_shift,
            .left_align     = left_align,
            .big_endian     = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_I2S_DIN,
        },
    };

    err = i2s_channel_init_std_mode(*chan, &std_cfg);
    if (err != ESP_OK) {
        i2s_del_channel(*chan);
        *chan = NULL;
        return err;
    }
    err = i2s_channel_enable(*chan);
    return err;
}

/* Read a block and compute basic metrics: RMS and saturation fraction */
static esp_err_t evaluate_config(i2s_chan_handle_t chan, double *out_rms, double *out_sat_frac, size_t *out_words_read)
{
    // allocate evaluation buffer on heap (simple and safe)
    int32_t *buf = heap_caps_malloc(BUFFER_WORDS * sizeof(int32_t), MALLOC_CAP_8BIT);
    if (!buf) return ESP_ERR_NO_MEM;
    size_t bytes_read = 0;
    esp_err_t r = i2s_channel_read(chan, buf, BUFFER_WORDS * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(200));
    if (r != ESP_OK) {
        free(buf);
        return r;
    }
    size_t words = bytes_read / sizeof(int32_t);
    double accum = 0.0;
    size_t sat = 0;
    for (size_t i = 0; i < words; ++i) {
        uint32_t w = (uint32_t)buf[i];
        // extract 24-bit sample as many INMP441 present MSB aligned in top bits usually
        uint32_t s24 = (w >> 8) & 0x00FFFFFFu;
        int32_t s = (int32_t)s24;
        if (s24 & 0x00800000u) s |= 0xFF000000u;
        double v = (double)s;
        accum += v * v;
        if ((s > 0 && (uint32_t)s > 0x007F0000u) || (s < 0 && (uint32_t)(-s) > 0x007F0000u)) sat++;
    }
    double rms = 0.0;
    if (words) rms = sqrt(accum / (double)words);
    *out_rms = rms;
    *out_sat_frac = words ? ((double)sat / (double)words) : 1.0;
    *out_words_read = words;
    free(buf);
    return ESP_OK;
}

/* The USB input callback: reads I2S and converts left samples to 32-bit PCM */
static esp_err_t my_input_cb(uint8_t *data, size_t max_len, size_t *bytes_written, void *arg)
{
    // fixed-size buffer with compile-time size
    static int32_t inbuf[IN_BUF_WORDS];
    size_t bytes_read = 0;
    esp_err_t ret = i2s_channel_read(rx_chan, inbuf, sizeof(inbuf), &bytes_read, pdMS_TO_TICKS(100));
    if (ret != ESP_OK || bytes_read == 0) {
        *bytes_written = 0;
        return ret == ESP_OK ? ESP_ERR_TIMEOUT : ret;
    }
    size_t words = bytes_read / sizeof(int32_t);
    // ensure we have LR pairs (stereo slots)
    if (words < 2) {
        *bytes_written = 0;
        return ESP_OK;
    }
    if (words & 1) words--; // make even
    size_t left_samples = words / 2;
    size_t to_send = left_samples * sizeof(int32_t);
    if (to_send > max_len) {
        left_samples = max_len / sizeof(int32_t);
        to_send = left_samples * sizeof(int32_t);
    }
    int32_t *out32 = (int32_t *)data;
    for (size_t i = 0, j = 0; j < left_samples; i += 2, ++j) {
        uint32_t raw = (uint32_t)inbuf[i];
        uint32_t s24 = (raw >> 8) & 0x00FFFFFFu;
        int32_t s = (int32_t)s24;
        if (s24 & 0x00800000u) s |= 0xFF000000u;
        out32[j] = s << 8; // left-align 24->32
    }
    *bytes_written = to_send;
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting auto-detect for INMP441 on pins BCLK=%d WS=%d DIN=%d", PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN);
    dbg_lock = xSemaphoreCreateMutex();

    bool left_align_opts[2] = { false, true };
    bool bit_shift_opts[2] = { false, true };
    int sample_rates[SAMPLE_RATE_TRIES] = {48000, 16000};

    bool found = false;
    bool sel_left_align=false, sel_bit_shift=false;
    int sel_sr = 48000;

    for (int sr_idx = 0; sr_idx < SAMPLE_RATE_TRIES && !found; ++sr_idx) {
        int sr = sample_rates[sr_idx];
        for (int la = 0; la < 2 && !found; ++la) {
            for (int bs = 0; bs < 2 && !found; ++bs) {
                ESP_LOGI(TAG, "Trying config: sr=%d left_align=%s bit_shift=%s", sr,
                         left_align_opts[la] ? "LEFT" : "STD", bit_shift_opts[bs] ? "YES" : "NO");

                if (rx_chan) {
                    i2s_channel_disable(rx_chan);
                    i2s_del_channel(rx_chan);
                    rx_chan = NULL;
                }
                esp_err_t err = i2s_init_std(&rx_chan, left_align_opts[la], bit_shift_opts[bs], sr);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "i2s init failed (%d), skipping", err);
                    continue;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                double rms=0.0, sat=1.0; size_t words=0;
                err = evaluate_config(rx_chan, &rms, &sat, &words);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "evaluate_config err=%d", err);
                    i2s_channel_disable(rx_chan);
                    i2s_del_channel(rx_chan);
                    rx_chan = NULL;
                    continue;
                }
                ESP_LOGI(TAG, "Eval sr=%d la=%d bs=%d words=%" PRIu32 " rms=%.1f sat_frac=%.3f",
                         sr, (int)left_align_opts[la], (int)bit_shift_opts[bs], (uint32_t)words, rms, sat);
                if (words >= 128 && rms > 50.0 && rms < 7000000.0 && sat < 0.10) {
                    found = true;
                    sel_left_align = left_align_opts[la];
                    sel_bit_shift = bit_shift_opts[bs];
                    sel_sr = sr;
                    ESP_LOGI(TAG, "Selected config: sr=%d left_align=%s bit_shift=%s (rms=%.1f sat=%.3f)",
                             sel_sr, sel_left_align ? "LEFT" : "STD", sel_bit_shift ? "YES" : "NO", rms, sat);
                    break;
                } else {
                    i2s_channel_disable(rx_chan);
                    i2s_del_channel(rx_chan);
                    rx_chan = NULL;
                }
            }
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "No good config auto-detected; defaulting to 48k, standard I2S, no bit_shift");
        if (rx_chan) { i2s_channel_disable(rx_chan); i2s_del_channel(rx_chan); rx_chan = NULL; }
        ESP_ERROR_CHECK(i2s_init_std(&rx_chan, false, false, 48000));
    } else {
        if (rx_chan) { i2s_channel_disable(rx_chan); i2s_del_channel(rx_chan); rx_chan = NULL; }
        ESP_ERROR_CHECK(i2s_init_std(&rx_chan, sel_left_align, sel_bit_shift, sel_sr));
    }

    uac_device_config_t cfg = {
        .input_cb = my_input_cb,
        .output_cb = NULL,
    };
    esp_err_t e = uac_device_init(&cfg);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "uac_device_init failed: %d", e);
    } else {
        ESP_LOGI(TAG, "uac_device initialized. Open host audio input device to receive stream.");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
