/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(motor_common, CONFIG_MOTOR_LOG_LEVEL);

#define MOTOR_PIN DT_ALIAS_LED0_GPIOS_PIN
#define MOTOR_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER

struct motor_data {
    const struct device *gpio_dev;
};

void motor_send_thread(void *arg1, void *arg2, void *arg3) {
    
}

static int motor_init(const struct device *dev) {
    struct motor_data *data = dev->data;

    data->gpio_dev = device_get_binding(MOTOR_PORT);
    if (!data->gpio_dev) {
        LOG_ERR("Failed to get binding for port %s", MOTOR_PORT);
        return -ENODEV;
    }

    int ret = gpio_pin_configure(data->gpio_dev, MOTOR_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure pin %d", MOTOR_PIN);
        return ret;
    }

    LOG_INF("Motor driver initialized");
    return 0;
}

static const struct motor_driver_api {
    // Define motor driver API functions here
} motor_driver_api;

static struct motor_data motor_driver_data;

DEVICE_DEFINE(motor_driver, "MOTOR_DRIVER", motor_init, NULL, &motor_driver_data, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &motor_driver_api);