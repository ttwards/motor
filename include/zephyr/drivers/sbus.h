/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SBUS_H
#define SBUS_H

#include "zephyr/toolchain.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

// SBUS API 函数指针定义
typedef void (*sbus_api_parseframe_t)(const struct device *dev);
typedef float (*sbus_api_getchannel_percentage_t)(const struct device *dev, uint8_t channelid);
typedef int (*sbus_api_getchannel_digital_t)(const struct device *dev, uint8_t channelid);

// SBUS 驱动 API 结构体
__subsystem struct sbus_driver_api {
    sbus_api_getchannel_percentage_t getchannel_percentage;
    sbus_api_getchannel_digital_t    getchannel_digital;
};

// 系统调用声明
__syscall float sbus_get_percent(const struct device *dev, uint8_t channelid);
__syscall int   sbus_get_digit(const struct device *dev, uint8_t channelid);

// 实现函数

static inline float z_impl_sbus_get_percent(const struct device *dev, uint8_t channelid) {
    const struct sbus_driver_api *api = (const struct sbus_driver_api *)dev->api;
    if (api->getchannel_percentage) {
        float temp = api->getchannel_percentage(dev, channelid);
        return temp;
    }
    return 0.0f;
}

static inline int z_impl_sbus_get_digit(const struct device *dev, uint8_t channelid) {
    const struct sbus_driver_api *api = (const struct sbus_driver_api *)dev->api;
    if (api->getchannel_digital) {
        return api->getchannel_digital(dev, channelid);
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/sbus.h>

#endif // SBUS_H