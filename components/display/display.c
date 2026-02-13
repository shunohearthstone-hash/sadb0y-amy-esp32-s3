#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "u8g2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "display";

static u8g2_t u8g2;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static uint8_t u8x8_byte_esp32_i2c(u8x8_t *u8x8,
                                   uint8_t msg,
                                   uint8_t arg_int,
                                   void *arg_ptr)
{
    switch (msg) {

    case U8X8_MSG_BYTE_INIT:
        return 1;

    case U8X8_MSG_BYTE_SEND:
        ESP_ERROR_CHECK(i2c_master_transmit(
            dev_handle,
            arg_ptr,
            arg_int,
            -1));
        return 1;

    case U8X8_MSG_BYTE_START_TRANSFER:
    case U8X8_MSG_BYTE_END_TRANSFER:
        return 1;
    }

    return 0;
}

static uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8,
                                         uint8_t msg,
                                         uint8_t arg_int,
                                         void *arg_ptr)
{
    switch (msg) {

    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        return 1;

    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(10);
        return 1;

    case U8X8_MSG_DELAY_100NANO:
        return 1;
    }

    return 0;
}

esp_err_t display_init(int sda_pin,
                       int scl_pin,
                       uint32_t i2c_freq_hz,
                       uint8_t i2c_addr)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = i2c_freq_hz,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        bus_handle,
        &dev_config,
        &dev_handle));

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_esp32_i2c,
        u8x8_gpio_and_delay_esp32);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

    ESP_LOGI(TAG, "SSD1306 initialized");

    return ESP_OK;
}

void display_clear(void)
{
    u8g2_ClearBuffer(&u8g2);
}

void display_draw_string(int x, int y, const char *str)
{
    u8g2_DrawStr(&u8g2, x, y, str);
}

void display_printf(int x, int y, const char *fmt, ...)
{
    char buffer[128];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    u8g2_DrawStr(&u8g2, x, y, buffer);
}

void display_update(void)
{
    u8g2_SendBuffer(&u8g2);
}

void display_set_contrast(uint8_t value)
{
    u8g2_SetContrast(&u8g2, value);
}

void display_power(bool on)
{
    u8g2_SetPowerSave(&u8g2, on ? 0 : 1);
}
