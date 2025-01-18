/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STEERWHEEL_H
#define STEERWHEEL_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>

#define RADPS_TO_RPM 9.54929659f

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

typedef struct {
	float angle_offset;

	float wheel_radius;

	float pos_offset_x;
	float pos_offset_y;

	bool inverse_steer;
	bool inverse_wheel;

	const struct device *steer_motor;
	const struct device *wheel_motor;
} steerwheel_cfg_t;

typedef struct {
	float current_angle;
	float current_speed;

	float set_angle;
	float set_rpm;

	bool negative;
} steerwheel_data_t;

static inline void steerwheel_set_speed(const struct device *dev, float speed)
{
	steerwheel_data_t *data = dev->data;
	const steerwheel_cfg_t *cfg = dev->config;

	data->current_angle = motor_get_angle(cfg->steer_motor);
	data->current_speed = motor_get_speed(cfg->wheel_motor);

	if (data->current_speed < 0) {
		data->current_speed = -data->current_speed;
		data->current_angle =
			fmodf(data->current_angle + 180.0f - cfg->angle_offset, 360.0f);
	}
	data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->wheel_radius;
	if (data->negative) {
		data->set_rpm = -data->set_rpm;
	}
	motor_set_speed(cfg->wheel_motor, data->set_rpm);
}

static inline void steerwheel_set_angle(const struct device *dev, float angle)
{
	steerwheel_data_t *data = dev->data;
	const steerwheel_cfg_t *cfg = dev->config;

	data->current_angle = motor_get_angle(cfg->steer_motor);
	data->current_speed = motor_get_speed(cfg->wheel_motor);

	if (cfg->inverse_steer) {
		angle = -angle;
	}

	float target_angle = fmodf(angle + cfg->angle_offset, 360.0f);

	float delta_angle = data->current_angle - target_angle;
	data->set_angle = 0;

	bool prev_nega = data->negative;

	if (delta_angle > 0) {
		if (delta_angle < 90) {
			data->set_angle = target_angle;
			data->negative = false;
		} else if (delta_angle < 180) {
			data->set_angle = target_angle - 180.0f;
			data->negative = true;
		} else if (delta_angle < 270) {
			data->set_angle = target_angle - 180.0f;
			data->negative = true;
		} else {
			data->set_angle = target_angle;
			data->negative = false;
		}
	} else if (delta_angle < 0) {
		if (delta_angle > -90) {
			data->set_angle = target_angle;
			data->negative = false;
		} else if (delta_angle > -180) {
			data->set_angle = target_angle - 180.0f;
			data->negative = true;
		} else if (delta_angle > -270) {
			data->set_angle = target_angle - 180.0f;
			data->negative = true;
		} else {
			data->set_angle = target_angle;
			data->negative = false;
		}
	}

	if (data->current_speed < 0) {
		data->current_speed = -data->current_speed;
		data->current_angle =
			fmodf(data->current_angle + 180.0f - cfg->angle_offset, 360.0f);
	}

	motor_set_angle(cfg->steer_motor, data->set_angle);

	if (data->negative != prev_nega) {
		data->set_rpm = -data->set_rpm;
		motor_set_speed(cfg->wheel_motor, data->set_rpm);
	}
}

static inline float steerwheel_get_speed(const struct device *dev)
{
	steerwheel_data_t *data = dev->data;
	const steerwheel_cfg_t *cfg = dev->config;

	data->current_speed = motor_get_speed(cfg->wheel_motor);

	if (data->negative) {
		data->current_speed = -data->current_speed;
	}

	return data->current_speed;
}

static inline float steerwheel_get_angle(const struct device *dev)
{
	steerwheel_data_t *data = dev->data;
	const steerwheel_cfg_t *cfg = dev->config;

	data->current_angle = motor_get_angle(cfg->steer_motor);

	if (data->negative) {
		data->current_angle =
			fmodf(data->current_angle + 180.0f - cfg->angle_offset, 360.0f);
	} else {
		data->current_angle = fmodf(data->current_angle - cfg->angle_offset, 360.0f);
	}

	if (cfg->inverse_steer) {
		data->current_angle = -data->current_angle;
	}

	return data->current_angle;
}

#ifdef __cplusplus
}
#endif

#endif // STEERWHEEL_H