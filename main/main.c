/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "priv_i2c_u8g2.h"
#include "u8g2.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_task.h"
#include "amy.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "rotary_encoder.h"
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "main";

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/gpio_num.h"
#include <math.h>
/*----------------------------------------------GPIO/MACROS-----------------------------------------------------------*/
//i2s pins
#define CONFIG_I2S_BCLK 11 // 25
#define CONFIG_I2S_LRCLK 9
#define CONFIG_I2S_DIN 10
// This can be 32 bit, int32_t -- helpful for digital output to a i2s->USB teensy3 board


// Potentiometer ADC channels
#define CONFIG_POT_ADC_CHANNEL ADC_CHANNEL_5 // GPIO6 on ESP32-S3 (ADC1_CH5)
// Physical GPIO for the pot (matches ADC_CHANNEL_3 on ESP32-S3)
#define POT_GPIO_NUM GPIO_NUM_6
// Rotary encoder pins
#define ENCODER_PIN_A GPIO_NUM_40
#define ENCODER_PIN_B GPIO_NUM_41

#define u8g2_task_stack_size (8 * 1024) // 8KB stack for the u8g2 task

#define CHANGE_DELTA_HZ 5.0f // Minimum frequency change in Hz to update the synth parameters, to avoid excessive event scheduling



typedef int16_t i2s_sample_type;



adc_oneshot_unit_handle_t pot_adc_handle;
static i2c_u8g2_handle_t s_display;
static u8g2_t *s_u8g2 = NULL;
static volatile int s_last_pot_raw = 0;
static volatile float s_last_pot_freq_hz = 0.0f;
// Last frequency that was sent to AMY (used for thresholding)
static volatile float s_last_sent_pot_freq_hz = 0.0f;

static void u8g2_task_function(void *pvParameters);

/* helper from priv_i2c_u8g2 component */
extern void demo_shapes(u8g2_t *u8g2);

static void pot_log_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        ESP_LOGI(TAG, "ADC: %d, Freq: %.1f Hz", s_last_pot_raw, s_last_pot_freq_hz);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
    }
}

//encoder
static void encoder_task(void *pvParameters)
{
    rotary_encoder_handle_t enc = (rotary_encoder_handle_t)pvParameters;
    int32_t last_count = 0;
    for (;;) {
        int32_t count = rotary_encoder_get_count(enc);
        if (count != last_count) {
            ESP_LOGI(TAG, "encoder count=%ld (delta=%ld)", (long)count, (long)(count - last_count));
            last_count = count;
            // Example: translate count -> amy_event here (use amy_add_event)
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Poll at 50Hz
    }
}

static void encoder_init_task(void *pvParameters)
{
    (void)pvParameters;
    // Delay init to avoid early-boot conflicts (PSRAM, console pins, etc.)
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[encoder_init] starting after delay");
    rotary_encoder_config_t enc_cfg = rotary_encoder_default_config(ENCODER_PIN_A, ENCODER_PIN_B);
    rotary_encoder_handle_t enc = NULL;
    esp_err_t err = rotary_encoder_new_with_config(&enc_cfg, &enc);
    ESP_LOGI(TAG, "[encoder_init] rotary_encoder_new_with_config returned %d", err);
    if (err == ESP_OK && enc) {
        xTaskCreate(encoder_task, "encoder_task", 4096, enc, 5, NULL);
    }

    vTaskDelete(NULL);
}

// I2C recover sequence will be performed inline in app_main.


// AMY synth states
extern struct state amy_global;

static void esp_show_debug(uint8_t t) {
    (void)t;
}

static amy_err_t setup_pot_adc(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &pot_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(pot_adc_handle, CONFIG_POT_ADC_CHANNEL, &chan_cfg));
    // Configure the GPIO pull to avoid floating inputs when the pot is
    // physically disconnected. This biases the input to a known state
    // and prevents unsafe sudden jumps in readings.
    gpio_set_direction(POT_GPIO_NUM, GPIO_MODE_INPUT);
    gpio_set_pull_mode(POT_GPIO_NUM, GPIO_PULLDOWN_ONLY);
    return AMY_OK;
}

static void start_test_tone(uint32_t start) {
    amy_event e = amy_default_event();
    e.time = start;
    e.osc = 0;
    e.wave = SINE;
    e.velocity = 0.7f;
    e.freq_coefs[0] = 220.0f;
    amy_add_event(&e);
}

/* 
static void update_tone_effect_from_pot(uint32_t now) {
    // Deprecated: function kept for compatibility but not used when pot_reader_task is active.
    (void)now;
}
*/

// Task: read the potentiometer, compute frequency and only trigger AMY update
// when the frequency change exceeds CHANGE_DELTA_HZ. This acts as the
// FreeRTOS "trigger" the user requested.
static void pot_reader_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        int raw = 0;
        if (adc_oneshot_read(pot_adc_handle, CONFIG_POT_ADC_CHANNEL, &raw) != ESP_OK) {
            // ADC failed, keep previous readings
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Quick stability check: take a few fast samples and ensure the
        // value isn't wildly fluctuating (which indicates a floating/unconnected pot).
        bool unstable = false;
        for (int i = 0; i < 2; ++i) {
            int tmp = 0;
            if (adc_oneshot_read(pot_adc_handle, CONFIG_POT_ADC_CHANNEL, &tmp) != ESP_OK) {
                unstable = true;
                break;
            }
            if (abs(tmp - raw) > 50) { // if samples vary more than ~50 counts, treat as unstable
                unstable = true;
                break;
            }
            raw = tmp;
        }
        if (unstable) {
            // Skip this cycle; keep last known good values to avoid sudden jumps
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float normalized = (float)raw / 4095.0f;
        float freq_hz = 110.0f + (normalized * 770.0f);

        // Update last-seen values for logging/UI
        s_last_pot_raw = raw;
        s_last_pot_freq_hz = freq_hz;

        // Compare against last sent frequency and only trigger AMY when change
        // exceeds the configured threshold (CHANGE_DELTA_HZ)
        if (fabsf(freq_hz - s_last_sent_pot_freq_hz) > CHANGE_DELTA_HZ) {
            // Prepare and send the AMY event at current sysclock.
            // Use a static event to avoid allocating a large struct on the task stack.
            static amy_event e;
            e = amy_default_event();
            e.time = amy_sysclock();
            e.osc = 0;
            e.freq_coefs[0] = freq_hz;
            amy_add_event(&e);

            // Remember the last value we sent
            s_last_sent_pot_freq_hz = freq_hz;
        }

        // Poll at a modest rate
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void u8g2_task_function(void *pvParameters)
{
    (void)pvParameters;
    if (s_u8g2 == NULL) {
        vTaskDelete(NULL);
        return;
    }

    u8g2_SetFont(s_u8g2, u8g2_font_6x10_tf);

    for (;;) {
        char line[32];
        u8g2_ClearBuffer(s_u8g2);

        u8g2_DrawStr(s_u8g2, 0, 12, "AMY Synth READY");

        snprintf(line, sizeof(line), "ADC: %d %.1fHz", s_last_pot_raw, s_last_pot_freq_hz);
        u8g2_DrawStr(s_u8g2, 0, 24, line);

        snprintf(line, sizeof(line), "I2S: B%d L%d D%d", CONFIG_I2S_BCLK, CONFIG_I2S_LRCLK, CONFIG_I2S_DIN);
        u8g2_DrawStr(s_u8g2, 0, 36, line);

        u8g2_SendBuffer(s_u8g2);

        vTaskDelay(pdMS_TO_TICKS(1000));

        // Show demo shapes for 1 second
      // demo_shapes(s_u8g2);
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

    void app_main(void)
{
   
    ESP_LOGI(TAG, "Hello world!");
/*
    // I2C recover: Flush Sequence ensure SDA released and toggle SCL 9 times
    printf("[startup] before i2c_recover\n");
    gpio_set_direction(CONFIG_I2C_U8G2_SDA_GPIO, GPIO_MODE_INPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CONFIG_I2C_U8G2_SCL_GPIO,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(CONFIG_I2C_U8G2_SCL_GPIO, 0);
        ets_delay_us(5);
        gpio_set_level(CONFIG_I2C_U8G2_SCL_GPIO, 1);
        ets_delay_us(5);
    }
    printf("[startup] after i2c_recover\n");
*/
    ESP_LOGI(TAG, "[startup] before i2c_u8g2_init");
    // Display configuration is now managed through menuconfig (Kconfig)
    i2c_u8g2_config_t display_cfg = i2c_u8g2_config_default();
    ESP_LOGI(TAG, "[startup] display i2c cfg: SDA=%d SCL=%d Freq=%d Addr=0x%02X Timeout=%dms",
             (int)display_cfg.sda_io_num,
             (int)display_cfg.scl_io_num,
             (int)display_cfg.scl_speed_hz,
             (unsigned int)display_cfg.device_address,
             (int)display_cfg.timeout_ms);

    esp_err_t display_err = i2c_u8g2_init(&s_display, &display_cfg);
    if (display_err != ESP_OK) {
        ESP_LOGE(TAG, "[startup] i2c_u8g2_init failed: %s", esp_err_to_name(display_err));
        return;
    }
    s_u8g2 = i2c_u8g2_get_u8g2(&s_display);
    if (s_u8g2 == NULL) {
        ESP_LOGE(TAG, "[startup] i2c_u8g2_get_u8g2 returned NULL");
        return;
    }
    ESP_LOGI(TAG, "[startup] after i2c_u8g2_init");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
        return;
    }

    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    // Configure and start AMY
    amy_config_t amy_cfg = amy_default_config();
    amy_cfg.audio = AMY_AUDIO_IS_I2S;
    amy_cfg.i2s_bclk = CONFIG_I2S_BCLK;
    amy_cfg.i2s_lrc = CONFIG_I2S_LRCLK;
    amy_cfg.i2s_dout = CONFIG_I2S_DIN;
    amy_cfg.i2s_din = -1;
    amy_cfg.i2s_mclk = -1;
    ESP_LOGI(TAG, "Starting AMY synth engine... (audio=%d, Fs=%d)", amy_cfg.audio, AMY_SAMPLE_RATE);
    ESP_LOGI(TAG, "[startup] before amy_start");
    amy_start(amy_cfg);
    ESP_LOGI(TAG, "[startup] after amy_start");

        // Setup rotary encoder
    // Defer rotary encoder initialization to a task to avoid early-boot conflicts
    xTaskCreate(encoder_init_task, "encoder_init_task", 2048, NULL, 5, NULL);


    // Setup ADC for frequency pot
    ESP_LOGI(TAG, "[startup] before setup_pot_adc");
    setup_pot_adc();
    ESP_LOGI(TAG, "[startup] after setup_pot_adc");
    
    uint32_t start_time = amy_sysclock();
    amy_reset_oscs();

    ESP_LOGI(TAG, "Scheduling test tone on OSC 0...");
    start_test_tone(start_time + 200);

    xTaskCreate(
        u8g2_task_function,
         "u8g2_task_function",
         u8g2_task_stack_size,
         NULL,
         5,
         NULL);
    xTaskCreate(
        pot_log_task,
         "pot_log_task",
         2048,
         NULL,
         4
         ,
         NULL);

    // Start pot reader task which will trigger AMY updates only when the
    // frequency change exceeds CHANGE_DELTA_HZ.
    xTaskCreate(
        pot_reader_task,
        "pot_reader_task",
        2048,
        NULL,
        4,
        NULL);

    ESP_LOGI(TAG, "Main loop started: pot reader task running");
    // Idle loop; pot_reader_task handles all pot->synth updates.
    while (1) {
        ESP_LOGI(TAG, "Main loop idle..."); // Log to show we're alive; actual work is in tasks.
        vTaskDelay(pdMS_TO_TICKS(5000));        }
    }

