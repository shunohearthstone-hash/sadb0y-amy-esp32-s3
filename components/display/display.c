#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "u8g2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "display";

static u8g2_t u8g2;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static display_i2c_mode_t active_i2c_mode = DISPLAY_I2C_MODE_HW;
static int sw_i2c_sda_pin = -1;
static int sw_i2c_scl_pin = -1;

// U8g2 expects the I2C byte callback to handle DC (command vs data) by
// prepending a control byte (0x00 for command, 0x40 for data) and sending data
// between START_TRANSFER and END_TRANSFER.
static uint8_t i2c_dc_state;
static uint8_t i2c_tx_buf[128];
static size_t i2c_tx_len;

static bool i2c_flush(void)
{
    if (i2c_tx_len == 0) {
        return true;
    }

    // Full-frame updates at 100kHz can be close to 100ms; allow headroom.
    esp_err_t err = i2c_master_transmit(dev_handle, i2c_tx_buf, i2c_tx_len, pdMS_TO_TICKS(300));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C transmit failed (%s), len=%u", esp_err_to_name(err), (unsigned)i2c_tx_len);
        return false;
    }

    return true;
}

static uint8_t u8x8_byte_esp32_i2c(u8x8_t *u8x8,
                                   uint8_t msg,
                                   uint8_t arg_int,
                                   void *arg_ptr)
{
    (void)u8x8;

    switch (msg) {

    case U8X8_MSG_BYTE_INIT:
        return 1;

    case U8X8_MSG_BYTE_SET_DC:
        // 0 = command, 1 = data
        i2c_dc_state = (arg_int ? 1 : 0);
        return 1;

    case U8X8_MSG_BYTE_START_TRANSFER:
        i2c_tx_len = 0;
        // Control byte required by SSD1306 over I2C
        i2c_tx_buf[i2c_tx_len++] = i2c_dc_state ? 0x40 : 0x00;
        return 1;

    case U8X8_MSG_BYTE_SEND:
        if (arg_int == 0) {
            return 1;
        }

        {
            const uint8_t *data = (const uint8_t *)arg_ptr;
            while (arg_int > 0) {
                size_t space = sizeof(i2c_tx_buf) - i2c_tx_len;
                if (space == 0) {
                    if (!i2c_flush()) {
                        return 0;
                    }
                    // Restart next chunk with control byte
                    i2c_tx_len = 0;
                    i2c_tx_buf[i2c_tx_len++] = i2c_dc_state ? 0x40 : 0x00;
                    space = sizeof(i2c_tx_buf) - i2c_tx_len;
                }

                size_t to_copy = (arg_int < space) ? arg_int : space;
                memcpy(&i2c_tx_buf[i2c_tx_len], data, to_copy);
                i2c_tx_len += to_copy;
                data += to_copy;
                arg_int -= (uint8_t)to_copy;
            }
        }

        return 1;

    case U8X8_MSG_BYTE_END_TRANSFER:
        if (!i2c_flush()) {
            return 0;
        }
        i2c_tx_len = 0;
        return 1;
    }

    return 0;
}

static uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8,
                                         uint8_t msg,
                                         uint8_t arg_int,
                                         void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg) {

    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        if (active_i2c_mode == DISPLAY_I2C_MODE_SW) {
            if (sw_i2c_sda_pin < 0 || sw_i2c_scl_pin < 0) {
                return 0;
            }

            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << sw_i2c_sda_pin) | (1ULL << sw_i2c_scl_pin),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&io_conf);
        }
        return 1;

    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        return 1;

    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(10);
        return 1;

    case U8X8_MSG_DELAY_100NANO:
        return 1;

    case U8X8_MSG_DELAY_I2C:
        esp_rom_delay_us(arg_int);
        return 1;

    case U8X8_MSG_GPIO_I2C_CLOCK:
        if (active_i2c_mode == DISPLAY_I2C_MODE_SW && sw_i2c_scl_pin >= 0) {
            gpio_num_t scl = (gpio_num_t)sw_i2c_scl_pin;
            if (arg_int == 0) {
                gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
                gpio_set_level(scl, 0);
            } else {
                gpio_set_direction(scl, GPIO_MODE_INPUT);
            }
            return 1;
        }
        break;

    case U8X8_MSG_GPIO_I2C_DATA:
        if (active_i2c_mode == DISPLAY_I2C_MODE_SW && sw_i2c_sda_pin >= 0) {
            gpio_num_t sda = (gpio_num_t)sw_i2c_sda_pin;
            if (arg_int == 0) {
                gpio_set_direction(sda, GPIO_MODE_OUTPUT_OD);
                gpio_set_level(sda, 0);
            } else {
                gpio_set_direction(sda, GPIO_MODE_INPUT);
            }
            return 1;
        }
        break;
    }

    return 0;
}

esp_err_t display_init_with_mode(int sda_pin,
                                 int scl_pin,
                                 uint32_t i2c_freq_hz,
                                 uint8_t i2c_addr,
                                 display_i2c_mode_t mode)
{
    active_i2c_mode = mode;
    sw_i2c_sda_pin = sda_pin;
    sw_i2c_scl_pin = scl_pin;

    esp_err_t probe_err = ESP_OK;

    if (mode == DISPLAY_I2C_MODE_HW) {
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

        probe_err = i2c_master_probe(bus_handle, i2c_addr, 100);
        if (probe_err != ESP_OK) {
            ESP_LOGW(TAG, "Device not found at 0x%02X (%s). Check wiring and pull-ups!", i2c_addr, esp_err_to_name(probe_err));

            ESP_LOGI(TAG, "Scanning I2C bus...");
            for (uint8_t i = 1; i < 127; i++) {
                if (i2c_master_probe(bus_handle, i, 50) == ESP_OK) {
                    ESP_LOGI(TAG, "Found device at 0x%02X", i);
                }
            }
        } else {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", i2c_addr);
        }
    } else {
        ESP_LOGI(TAG, "Using software I2C on SDA=%d SCL=%d", sda_pin, scl_pin);
    }

    u8g2_Setup_ssd1315_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        mode == DISPLAY_I2C_MODE_SW ? u8x8_byte_sw_i2c : u8x8_byte_esp32_i2c,
        u8x8_gpio_and_delay_esp32);

    u8g2_SetI2CAddress(&u8g2, (uint8_t)(i2c_addr << 1));

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

    if (probe_err == ESP_OK) {
        ESP_LOGI(TAG, "SSD1315 initialized");
    } else {
        ESP_LOGW(TAG, "SSD1315 init completed without ACK at 0x%02X", i2c_addr);
    }

    return ESP_OK;
}

esp_err_t display_init(int sda_pin,
                       int scl_pin,
                       uint32_t i2c_freq_hz,
                       uint8_t i2c_addr)
{
    return display_init_with_mode(sda_pin, scl_pin, i2c_freq_hz, i2c_addr, DISPLAY_I2C_MODE_HW);
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
