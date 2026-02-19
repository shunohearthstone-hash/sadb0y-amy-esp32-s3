idf_component_register(SRCS
    "main.c"
    PRIV_REQUIRES spi_flash esp_driver_i2s esp_driver_i2c esp_adc esp_driver_uart esp_timer display rotary_encoder
    REQUIRES amy 
    INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_TARGET} PUBLIC
    -Wno-uninitialized
    -DESP_PLATFORM
)

# `amy.c` needs relaxed strict-aliasing warnings; handled by the `amy` component.