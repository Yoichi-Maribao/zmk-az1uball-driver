/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT palette_az1uball

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <math.h>
#include "az1uball.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(az1uball, LOG_LEVEL_DBG);

volatile uint8_t AZ1UBALL_MOUSE_MAX_SPEED = 25;
volatile uint8_t AZ1UBALL_MOUSE_MAX_TIME = 5;
volatile float AZ1UBALL_MOUSE_SMOOTHING_FACTOR = 1.3f;
volatile uint8_t AZ1UBALL_SCROLL_MAX_SPEED = 1;
volatile uint8_t AZ1UBALL_SCROLL_MAX_TIME = 1;
volatile float AZ1UBALL_SCROLL_SMOOTHING_FACTOR = 0.5f;
volatile float AZ1UBALL_HUE_INCREMENT_FACTOR = 0.3f;

#define POLL_INTERVAL K_MSEC(10)  // Polling interval

enum az1uball_mode {
    AZ1UBALL_MODE_MOUSE,
    AZ1UBALL_MODE_SCROLL
};

static enum az1uball_mode current_mode = AZ1UBALL_MODE_MOUSE;

static int previous_x = 0;
static int previous_y = 0;

void az1uball_toggle_mode(void) {
    current_mode = (current_mode == AZ1UBALL_MODE_MOUSE) ? AZ1UBALL_MODE_SCROLL : AZ1UBALL_MODE_MOUSE;
    // Optional: Add logging or LED indication here to show the current mode
    LOG_DBG("AZ1UBALL mode switched to %s", (current_mode == AZ1UBALL_MODE_MOUSE) ? "MOUSE" : "SCROLL");
}

static void az1uball_process_movement(struct az1uball_data *data, int delta_x, int delta_y, uint32_t time_between_interrupts, int max_speed, int max_time, float smoothing_factor) {
    float scaling_factor = 1.0f;
    if (time_between_interrupts < max_time) {
        // Exponential scaling calculation
        float exponent = -3.0f * (float)time_between_interrupts / max_time; // Adjust -3.0f for desired curve
        scaling_factor = 1.0f + (max_speed - 1.0f) * expf(exponent);
    }

    // Apply scaling based on mode
    if (current_mode == AZ1UBALL_MODE_SCROLL) {
        scaling_factor *= 2.5f; // Example: Increase scaling for scroll mode
    }

    /* Accumulate deltas atomically */
    atomic_add(&data->x_buffer, delta_x);
    atomic_add(&data->y_buffer, delta_y);

    int scaled_x_movement = (int)(delta_x * scaling_factor);
    int scaled_y_movement = (int)(delta_y * scaling_factor);

    // Apply smoothing
    data->smoothed_x = (int)(smoothing_factor * scaled_x_movement + (1.0f - smoothing_factor) * previous_x);
    data->smoothed_y = (int)(smoothing_factor * scaled_y_movement + (1.0f - smoothing_factor) * previous_y);

    data->previous_x = data->smoothed_x;
    data->previous_y = data->smoothed_y;
}

/* Execution functions for asynchronous work */
void az1uball_read_data_work(struct k_work *work)
{
    struct az1uball_data *data = CONTAINER_OF(work, struct az1uball_data, work);
    const struct az1uball_config *config = data->dev->config;
    uint8_t buf[5];  //Buffer to store X, Y, Switch data
    int ret;

    // Read data from I2C
    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, sizeof(buf));
    if (ret) {
        LOG_ERR("Failed to read movement data from AZ1YBALL: %d", ret);
        return;
    }

    uint32_t time_between_interrupts;

    k_mutex_lock(&data->data_lock, K_FOREVER);
    time_between_interrupts = data->last_interrupt_time - data->previous_interrupt_time;
    k_mutex_unlock(&data->data_lock);

    /* Calculate deltas */
    int16_t delta_x = (int16_t)buf[1] - (int16_t)buf[0]; // RIGHT - LEFT
    int16_t delta_y = (int16_t)buf[3] - (int16_t)buf[2]; // DOWN - UP

    /* Report movement immediately if non-zero */
    if (delta_x != 0 || delta_y != 0) {
        if (current_mode == AZ1UBALL_MODE_MOUSE) {
            az1uball_process_movement(data, delta_x, delta_y, time_between_interrupts, AZ1UBALL_MOUSE_MAX_SPEED, AZ1UBALL_MOUSE_MAX_TIME, AZ1UBALL_MOUSE_SMOOTHING_FACTOR);

            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_X, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_Y, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        } else if (current_mode == AZ1UBALL_MODE_SCROLL) {
            az1uball_process_movement(data, delta_x, delta_y, time_between_interrupts, AZ1UBALL_SCROLL_MAX_SPEED, AZ1UBALL_SCROLL_MAX_TIME, AZ1UBALL_SCROLL_SMOOTHING_FACTOR);

            /* Report relative X movement */
            if (delta_x != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_WHEEL, data->smoothed_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", data->smoothed_x);
                }
            }

            /* Report relative Y movement */
            if (delta_y != 0) {
                ret = input_report_rel(data->dev, INPUT_REL_HWHEEL, data->smoothed_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", data->smoothed_y);
                }
            }
        }
    }

    /* Update switch state */
    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    /* Report switch state if it changed */
    if (data->sw_pressed != data->sw_pressed_prev) {
        ret = input_report_key(data->dev, INPUT_BTN_0, data->sw_pressed ? 1 : 0, true, K_NO_WAIT);
        if (ret) {
            LOG_ERR("Failed to report key");
        } else {
            LOG_DBG("Reported key");
        }

        LOG_DBG("Reported switch state: %d", data->sw_pressed);

        data->sw_pressed_prev = data->sw_pressed;
    }

    /* Clear movement registers */
    uint8_t zero = 0;
    i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_UP, zero);
    i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, zero);
}

static void az1uball_polling(struct k_timer *timer)
{
    struct az1uball_data *data = CONTAINER_OF(timer, struct az1uball_data, polling_timer);

    uint32_t current_time = k_uptime_get();

    k_mutex_lock(&data->data_lock, K_NO_WAIT);
    data->previous_interrupt_time = data->last_interrupt_time;
    data->last_interrupt_time = current_time;
    k_mutex_unlock(&data->data_lock);

    /* Schedule the work item to handle the interrupt in thread context */
    k_work_submit(&data->work);
}

/* AZ1UBALLの初期化 */
static int az1uball_init(const struct device *dev)
{
    const struct az1uball_config *config = dev->config;
    struct az1uball_data *data = dev->data;
    int ret;

    LOG_INF("AZ1UBALL driver initializing");

    k_work_init(&data->work, az1uball_read_data_work);
    k_timer_init(&data->polling_timer, az1uball_polling, NULL);
    k_timer_start(&data->polling_timer, POLL_INTERVAL, POLL_INTERVAL);

    /* Check if the I2C device is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    /* Set high speed mode */
    uint8_t cmd = 0x91;
    ret = i2c_write_dt(&config->i2c, &cmd, sizeof(cmd));
    if (ret) {
        LOG_ERR("Failed to set AZ mode");
        return ret;
    }

    return 0;
}

#define AZ1UBALL_DEFINE(n)                                           \
  static struct az1uball_data az1uball_data_##n;                     \
  static const struct az1uball_config az1uball_config_##n = {        \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                \
  };                                                                 \
  DEVICE_DT_INST_DEFINE(n,                                           \
                        az1uball_init,                               \
                        NULL,                                        \
                        &az1uball_data_##n,                          \
                        &az1uball_config_##n,                        \
                        POST_KERNEL,                                 \
                        CONFIG_INPUT_INIT_PRIORITY,                  \
                        NULL);

DT_INST_FOREACH_STATUS_OKAY(AZ1UBALL_DEFINE)