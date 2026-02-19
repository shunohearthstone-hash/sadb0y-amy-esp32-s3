#include "rotary_encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "rotary_encoder";

typedef struct rotary_encoder_s {
    pcnt_unit_handle_t pcnt_unit;
    QueueHandle_t event_queue;
} rotary_encoder_t;

static const int32_t default_watch_points[] = {
    ROTARY_ENCODER_DEFAULT_LOW_LIMIT,
    -50,
    0,
    50,
    ROTARY_ENCODER_DEFAULT_HIGH_LIMIT,
};

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t *edata,
                                    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)user_ctx, &edata->watch_point_value, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static esp_err_t rotary_encoder_create_unit(pcnt_unit_handle_t *unit, int32_t low_limit, int32_t high_limit)
{
    ESP_RETURN_ON_FALSE(unit != NULL, ESP_ERR_INVALID_ARG, TAG, "unit pointer is null");

    pcnt_unit_config_t unit_config = {
        .low_limit = low_limit,
        .high_limit = high_limit,
    };

    return pcnt_new_unit(&unit_config, unit);
}

static esp_err_t rotary_encoder_create_channels(pcnt_unit_handle_t unit, gpio_num_t pin_a, gpio_num_t pin_b)
{
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };

    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    return ESP_OK;
}

rotary_encoder_config_t rotary_encoder_default_config(gpio_num_t pin_a, gpio_num_t pin_b)
{
    rotary_encoder_config_t config = {
        .pin_a = pin_a,
        .pin_b = pin_b,
        .high_limit = ROTARY_ENCODER_DEFAULT_HIGH_LIMIT,
        .low_limit = ROTARY_ENCODER_DEFAULT_LOW_LIMIT,
        .watch_points = default_watch_points,
        .watch_point_count = sizeof(default_watch_points) / sizeof(default_watch_points[0]),
        .glitch_filter_ns = ROTARY_ENCODER_DEFAULT_GLITCH_NS,
        .event_queue_size = ROTARY_ENCODER_DEFAULT_QUEUE_SIZE,
    };
    return config;
}

esp_err_t rotary_encoder_new_with_config(const rotary_encoder_config_t *config, rotary_encoder_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid config");
    ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle pointer");

    rotary_encoder_t *encoder = calloc(1, sizeof(rotary_encoder_t));
    ESP_RETURN_ON_FALSE(encoder, ESP_ERR_NO_MEM, TAG, "no memory for encoder");

    ESP_ERROR_CHECK(rotary_encoder_create_unit(&encoder->pcnt_unit, config->low_limit, config->high_limit));
    if (config->glitch_filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = config->glitch_filter_ns,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder->pcnt_unit, &filter_config));
    }

    ESP_ERROR_CHECK(rotary_encoder_create_channels(encoder->pcnt_unit, config->pin_a, config->pin_b));

    if (config->watch_points && config->watch_point_count > 0) {
        for (size_t i = 0; i < config->watch_point_count; i++) {
            ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder->pcnt_unit, config->watch_points[i]));
        }
    }

    size_t queue_size = config->event_queue_size ? config->event_queue_size : ROTARY_ENCODER_DEFAULT_QUEUE_SIZE;
    encoder->event_queue = xQueueCreate(queue_size, sizeof(int));
    if (!encoder->event_queue) {
        ESP_LOGE(TAG, "failed to create queue");
        goto err;
    }

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_on_reach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder->pcnt_unit, &cbs, encoder->event_queue));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder->pcnt_unit));

    *out_handle = encoder;
    return ESP_OK;

err:
    rotary_encoder_delete(encoder);
    return ESP_FAIL;
}

esp_err_t rotary_encoder_new(gpio_num_t pin_a, gpio_num_t pin_b, rotary_encoder_handle_t *out_handle)
{
    rotary_encoder_config_t config = rotary_encoder_default_config(pin_a, pin_b);
    return rotary_encoder_new_with_config(&config, out_handle);
}

int32_t rotary_encoder_get_count(rotary_encoder_handle_t handle)
{
    if (!handle) return 0;
    int count = 0;
    pcnt_unit_get_count(handle->pcnt_unit, &count);
    return count;
}

esp_err_t rotary_encoder_reset(rotary_encoder_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return pcnt_unit_clear_count(handle->pcnt_unit);
}

QueueHandle_t rotary_encoder_get_event_queue(rotary_encoder_handle_t handle)
{
    return handle ? handle->event_queue : NULL;
}

esp_err_t rotary_encoder_delete(rotary_encoder_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    if (handle->pcnt_unit) {
        pcnt_unit_stop(handle->pcnt_unit);
        pcnt_unit_disable(handle->pcnt_unit);
    }
    if (handle->event_queue) {
        vQueueDelete(handle->event_queue);
    }
    free(handle);
    return ESP_OK;
}