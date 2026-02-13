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
#include "esp_log.h"
#include "esp_err.h"

#include <stdlib.h>

#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

#define CONFIG_I2S_BCLK 12 // 25
#define CONFIG_I2S_LRCLK 11
#define CONFIG_I2S_DIN 10

#define CONFIG_I2C_SDA 8
#define CONFIG_I2C_SCL 9
#define SSD1306_I2C_ADDR 0x3C

#define CONFIG_POT_ADC_CHANNEL ADC_CHANNEL_3 // GPIO4 on ESP32-S3

#define u8g2_task_stack_size (8 * 1024) // 8KB stack for the u8g2 task

// This can be 32 bit, int32_t -- helpful for digital output to a i2s->USB teensy3 board
#define I2S_SAMPLE_TYPE I2S_BITS_PER_SAMPLE_16BIT
typedef int16_t i2s_sample_type;


// MUTEX for delta queue
SemaphoreHandle_t xQueueSemaphore;
adc_oneshot_unit_handle_t pot_adc_handle;

static void u8g2_task_function(void *pvParameters);

void delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}


// AMY synth states
extern struct state amy_global;

void esp_show_debug(uint8_t t) {
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
    int raw = 0;
    if (adc_oneshot_read(pot_adc_handle, CONFIG_POT_ADC_CHANNEL, &raw) != ESP_OK) {
        return;
    }

    float normalized = (float)raw / 4095.0f;
    float freq_hz = 110.0f + (normalized * 770.0f);

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
        display_printf(0, 12, "ESP-IDF 6");
        display_printf(0, 28, "Heap: %d", esp_get_free_heap_size());
        display_update();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    printf("Hello world!\n");

    display_init(CONFIG_I2C_SDA, CONFIG_I2C_SCL, 100000, SSD1306_I2C_ADDR);
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
    amy_cfg.i2s_bclk = CONFIG_I2S_BCLK;
    amy_cfg.i2s_lrc = CONFIG_I2S_LRCLK;
    amy_cfg.i2s_dout = CONFIG_I2S_DIN;
    amy_cfg.i2s_din = -1;
    amy_cfg.i2s_mclk = -1;
    amy_start(amy_cfg);

    // Setup ADC for frequency pot
    setup_pot_adc();

    int64_t start_time = amy_sysclock();
    amy_reset_oscs();

    start_test_tone(start_time + 200);

    xTaskCreate(u8g2_task_function, "u8g2_task_function", u8g2_task_stack_size, NULL, 0, NULL);

    // Spin this core, updating synth params from the pot.
    for (;;) {
        update_tone_effect_from_pot(amy_sysclock());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
