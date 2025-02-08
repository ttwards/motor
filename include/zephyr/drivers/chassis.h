/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "zephyr/drivers/pid.h"
#include "zephyr/toolchain.h"
#include "zephyr/zbus/zbus.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/steerwheel.h>

#ifndef RADS_TO_RPM
#define RADPS_TO_RPM 6.28318531f
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

#define CHASSIS_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)        \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

typedef struct {
	float speedX;
	float speedY;
	float gyro;
	float angle;
} chassis_status_t;

struct pos_data {
	float Yaw;
	float accel[3];
	float gyro[3];
};

typedef struct {
	float targetYaw;
	float currentYaw;

	float targetGyro;
	float currentGyro;

	float targetXSpeed;
	float targetYSpeed;
	float currentXSpeed;
	float currentYSpeed;

	uint32_t currTime;
	uint32_t prevTime;

	bool angleControl;

	float angle_to_center[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT];
	float distance_to_center[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT];

	chassis_status_t chassis_status;

	float pid_input;
	float static0;

	struct zbus_channel *chassis_sensor_zbus;
} chassis_data_t;

typedef struct {
	struct pid_data *angle_pid;
	const struct device *steerwheels[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT];
	float pos_X_offset[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT];
	float pos_Y_offset[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT];
} chassis_cfg_t;

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef void (*chassis_set_speed_t)(const struct device *dev, float speedX, float speedY);

typedef void (*chassis_set_angle_t)(const struct device *dev, float angle);

typedef void (*chassis_set_gyro_t)(const struct device *dev, float gyro);

typedef chassis_status_t *(*chassis_get_status_t)(const struct device *dev);

/**
 * @brief Servo Motor driver API
 */
__subsystem struct chassis_driver_api {
	chassis_set_speed_t set_speed;
	chassis_set_angle_t set_angle;
	chassis_set_gyro_t set_gyro;
	chassis_get_status_t get_status;
};

__syscall void chassis_set_speed(const struct device *dev, float speedX, float speedY);
static inline void z_impl_chassis_set_speed(const struct device *dev, float speedX, float speedY)
{
	const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;

	if (api->set_speed != NULL) {
		api->set_speed(dev, speedX, speedY);
	}
}

__syscall void chassis_set_angle(const struct device *dev, float angle);
static inline void z_impl_chassis_set_angle(const struct device *dev, float angle)
{
	const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;

	if (api->set_angle != NULL) {
		api->set_angle(dev, angle);
	}
}

__syscall void chassis_set_gyro(const struct device *dev, float gyro);
static inline void z_impl_chassis_set_gyro(const struct device *dev, float gyro)
{
	const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;

	if (api->set_gyro != NULL) {
		api->set_gyro(dev, gyro);
	}
}

__syscall chassis_status_t *chassis_get_status(const struct device *dev);
static inline chassis_status_t *z_impl_chassis_get_status(const struct device *dev)
{
	const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;

	if (api->get_status != NULL) {
		return api->get_status(dev);
	}
	return NULL;
}

static void chassis_set_sensor_zbus(const struct device *dev, const struct zbus_channel *chan)
{
	chassis_data_t *data = dev->data;
	data->chassis_sensor_zbus = (struct zbus_channel *)chan;
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/chassis.h>

#endif // STEERWHEEL_H