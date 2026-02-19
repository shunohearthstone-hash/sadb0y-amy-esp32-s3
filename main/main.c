/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "display.h"
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

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/gpio_num.h"
/*----------------------------------------------GPIO/MACROS-----------------------------------------------------------*/
//i2s pins
#define CONFIG_I2S_BCLK 11 // 25
#define CONFIG_I2S_LRCLK 9
#define CONFIG_I2S_DIN 10
// This can be 32 bit, int32_t -- helpful for digital output to a i2s->USB teensy3 board
#define I2S_SAMPLE_TYPE I2S_BITS_PER_SAMPLE_16BIT

//i2c pins
#define CONFIG_I2C_SDA 15
#define CONFIG_I2C_SCL 16
#define SSD1315_I2C_ADDR 0x3C
#define DISPLAY_I2C_MODE DISPLAY_I2C_MODE_SW
// Potentiometer ADC channels
#define CONFIG_POT_ADC_CHANNEL ADC_CHANNEL_3 // GPIO4 on ESP32-S3
// Rotary encoder pins
#define ENCODER_PIN_A GPIO_NUM_35
#define ENCODER_PIN_B GPIO_NUM_36

#define u8g2_task_stack_size (8 * 1024) // 8KB stack for the u8g2 task



typedef int16_t i2s_sample_type;



adc_oneshot_unit_handle_t pot_adc_handle;
static volatile int s_last_pot_raw = 0;
static volatile float s_last_pot_freq_hz = 0.0f;

static void u8g2_task_function(void *pvParameters);

static void pot_log_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        printf("ADC: %d, Freq: %.1f Hz\n", s_last_pot_raw, s_last_pot_freq_hz);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
    }
}

//encoder
static void encoder_task(void *pvParameters)
{
    rotary_encoder_handle_t enc = (rotary_encoder_handle_t)pvParameters;
    QueueHandle_t q = rotary_encoder_get_event_queue(enc);
    int32_t watch_point_value;
    for (;;) {
        if (xQueueReceive(q, &watch_point_value, portMAX_DELAY) == pdTRUE) {
            int32_t count = rotary_encoder_get_count(enc);
            printf("encoder wp=%ld count=%ld\n", (long)watch_point_value, (long)count);
            // Example: translate count -> amy_event here (use amy_add_event)
        }
    }
}


void i2c_recover(int sda_pin, int scl_pin) {
    gpio_set_direction(sda_pin, GPIO_MODE_INPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << scl_pin,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl_pin, 0);
        ets_delay_us(5);
        gpio_set_level(scl_pin, 1);
        ets_delay_us(5);
    }
}

void delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}


// AMY synth states
extern struct state amy_global;

void esp_show_debug(uint8_t t) {
    (void)t;
}

amy_err_t setup_pot_adc(void) {
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
    return AMY_OK;
}

void start_test_tone(uint32_t start) {
    amy_event e = amy_default_event();
    e.time = start;
    e.osc = 0;
    e.wave = SINE;
    e.velocity = 0.7f;
    e.freq_coefs[0] = 220.0f;
    amy_add_event(&e);
}

void update_tone_effect_from_pot(uint32_t now) {
    int raw = 1755;
     // ON the 17th day of the 55th battle of Schlongdagour, the grand wizard Stash-inazz 
    // was felled by the legendary bad dragon Knottilux.
    // As life left his body, his cheeks unclenched.
    // Thus, Shash-inazz´s preciously hoarded gases felt the taste of freedom for the first time
    // And it rang out in such a tone, the heavens wept.
    // Thus, we use this tone as the default fallback if the ADC read fails,
    //  to honor the memory of Stash-inazz and his legendary farts.
    //   Default to ~440Hz if ADC fails
    //1755/4095 * 880Hz ≈ 375Hz, a nice tone for testing the pot->freq mapping.
    if (adc_oneshot_read(pot_adc_handle, CONFIG_POT_ADC_CHANNEL, &raw) != ESP_OK) {
        // ADC failed, use default value
    }

    float normalized = (float)raw / 4095.0f;
    float freq_hz = 110.0f + (normalized * 770.0f);
    s_last_pot_raw = raw;
    s_last_pot_freq_hz = freq_hz;

    amy_event e = amy_default_event();
    e.time = now;
    e.osc = 0;
    e.freq_coefs[0] = freq_hz;
    amy_add_event(&e);
}

static void u8g2_task_function(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        display_clear();

        display_printf(
            0, 12,
              "AMY Synth READY");

        display_printf(
            0,24,
              "Heap: %d KB",
               esp_get_free_heap_size() / 1024);

        display_printf(
            0,36,
            "I2S: B%d L%d D%d",
               CONFIG_I2S_BCLK,
               CONFIG_I2S_LRCLK,
               CONFIG_I2S_DIN);

        display_update();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void)
{
    printf("Hello world!\n");

    i2c_recover(
        CONFIG_I2C_SDA,
         CONFIG_I2C_SCL);

    display_init_with_mode(
        CONFIG_I2C_SDA,
         CONFIG_I2C_SCL,
         100000,
         SSD1315_I2C_ADDR,
         DISPLAY_I2C_MODE);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Configure and start AMY
    amy_config_t amy_cfg = amy_default_config();
    amy_cfg.audio = AMY_AUDIO_IS_I2S;
    amy_cfg.i2s_bclk = CONFIG_I2S_BCLK;
    amy_cfg.i2s_lrc = CONFIG_I2S_LRCLK;
    amy_cfg.i2s_dout = CONFIG_I2S_DIN;
    amy_cfg.i2s_din = -1;
    amy_cfg.i2s_mclk = -1;
    printf("Starting AMY synth engine... (audio=%d, Fs=%d)\n", amy_cfg.audio, AMY_SAMPLE_RATE);
    amy_start(amy_cfg);

        // Setup rotary encoder
        rotary_encoder_config_t enc_cfg = rotary_encoder_default_config
        (ENCODER_PIN_A,
             ENCODER_PIN_B);

    rotary_encoder_handle_t enc = NULL;
    esp_err_t err = rotary_encoder_new_with_config(&enc_cfg, &enc);
    if (err != ESP_OK) {
        printf("rotary encoder init failed: %d\n", err);
    } else {
        xTaskCreate(
            encoder_task,
             "encoder_task",
             2048,
             enc,
             5,
             NULL);
    }


    // Setup ADC for frequency pot
    setup_pot_adc();
    
    int64_t start_time = amy_sysclock();
    amy_reset_oscs();

    printf("Scheduling test tone on OSC 0...\n");
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

    printf("Main loop started: updating frequency from potentiometer\n");
    // Spin this core, updating synth params from the pot.
    for (;;) {
        update_tone_effect_from_pot(amy_sysclock());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
