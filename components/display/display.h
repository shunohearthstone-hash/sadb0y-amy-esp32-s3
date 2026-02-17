#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DISPLAY_I2C_MODE_HW = 0,
    DISPLAY_I2C_MODE_SW = 1,
} display_i2c_mode_t;

esp_err_t display_init(int sda_pin,
                       int scl_pin,
                       uint32_t i2c_freq_hz,
                       uint8_t i2c_addr);

esp_err_t display_init_with_mode(int sda_pin,
                                 int scl_pin,
                                 uint32_t i2c_freq_hz,
                                 uint8_t i2c_addr,
                                 display_i2c_mode_t mode);

void display_clear(void);
void display_printf(int x, int y, const char *fmt, ...);
void display_draw_string(int x, int y, const char *str);
void display_update(void);
void display_set_contrast(uint8_t value);
void display_power(bool on);

#ifdef __cplusplus
}
#endif
