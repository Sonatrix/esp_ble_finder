#include "clicker.h"

#include <math.h>
#include <string.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "proximity.h"

#define SAMPLE_RATE 16000
#define CLICK_SAMPLES 256
#define V2_TOUCH_ADDR 0x15
#define VOL_V1 70
#define VOL_V2 90

static const char *TAG = "clicker";
static esp_codec_dev_handle_t s_spk;
static int16_t s_click[CLICK_SAMPLES];
static int16_t s_scratch[CLICK_SAMPLES];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_enabled = true;
static bool s_fresh;
static int s_rssi;

static void make_click(void)
{
    for (int i = 0; i < CLICK_SAMPLES; ++i) {
        const float t = (float)i / (float)SAMPLE_RATE;
        const float env = (i < CLICK_SAMPLES / 2) ? 1.f : 1.f - (float)(i - CLICK_SAMPLES / 2) / (CLICK_SAMPLES / 2);
        s_click[i] = (int16_t)(sinf(2.f * 3.14159265f * 1000.f * t) * 20000.f * env);
    }
}

static void play_click(void)
{
    if (s_spk == NULL) {
        return;
    }
    memcpy(s_scratch, s_click, sizeof(s_click));
    if (esp_codec_dev_write(s_spk, s_scratch, sizeof(s_scratch)) != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "click write failed");
    }
}

static void clicker_task(void *arg)
{
    (void)arg;
    for (;;) {
        bool enabled;
        bool fresh;
        int rssi;
        taskENTER_CRITICAL(&s_lock);
        enabled = s_enabled;
        fresh = s_fresh;
        rssi = s_rssi;
        taskEXIT_CRITICAL(&s_lock);

        const int wait_ms = (enabled && fresh) ? proximity_click_ms(rssi)
                                               : PROXIMITY_CLICK_SLOWEST_MS;
        if (enabled && fresh) {
            play_click();
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

static bool board_is_v2(void)
{
    return i2c_master_probe(bsp_i2c_get_handle(), V2_TOUCH_ADDR, 50) == ESP_OK;
}

static esp_err_t speaker_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");

    i2s_chan_handle_t tx = NULL;
    i2s_chan_handle_t rx = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");

    const i2s_std_config_t i2s_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din = BSP_I2S_DSIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &i2s_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx enable");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &i2s_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx enable");

    audio_codec_i2s_cfg_t codec_i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .tx_handle = tx,
        .rx_handle = rx,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&codec_i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "i2s data_if");

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(i2c_ctrl_if, ESP_FAIL, TAG, "i2c ctrl");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_dev, ESP_FAIL, TAG, "es8311");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_dev,
        .data_if = data_if,
    };
    s_spk = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_spk, ESP_FAIL, TAG, "codec dev");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 1,
        .sample_rate = SAMPLE_RATE,
        .mclk_multiple = 256,
    };
    if (esp_codec_dev_open(s_spk, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "speaker open failed");
        s_spk = NULL;
        return ESP_FAIL;
    }

    const int volume = board_is_v2() ? VOL_V2 : VOL_V1;
    esp_codec_dev_set_out_vol(s_spk, volume);
    ESP_LOGI(TAG, "speaker volume %d", volume);
    return ESP_OK;
}

void clicker_start(void)
{
    make_click();
    if (speaker_init() != ESP_OK) {
        ESP_LOGE(TAG, "speaker init failed");
        s_spk = NULL;
        return;
    }
    play_click();
    if (xTaskCreate(clicker_task, "clicker", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "clicker task failed");
    }
}

void clicker_set_enabled(bool enabled)
{
    taskENTER_CRITICAL(&s_lock);
    s_enabled = enabled;
    taskEXIT_CRITICAL(&s_lock);
}

void clicker_update(bool fresh, int rssi)
{
    taskENTER_CRITICAL(&s_lock);
    s_fresh = fresh;
    s_rssi = rssi;
    taskEXIT_CRITICAL(&s_lock);
}
