/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/pid.h>
#include "zephyr/drivers/wheel.h"
#include <zephyr/logging/log.h>
#include "chassis.h"
#include "zephyr/kernel.h"
#include "arm_math.h"
#include "zephyr/sys/time_units.h"
#include <math.h>

#define DT_DRV_COMPAT ares_chassis

#define RAD2DEG(x) 57.2957795131f * x

LOG_MODULE_REGISTER(chassis, CONFIG_MOTOR_LOG_LEVEL);

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962f
#define M_PI_2 1.57079632679489661923f
#endif

int cchassis_init(const struct device *dev)
{
	chassis_data_t *data = dev->data;
	const chassis_cfg_t *cfg = dev->config;
	int idx = 0;
	while (cfg->wheels[idx] != NULL) {
		float arc;
		arm_atan2_f32(cfg->pos_Y_offset[idx], cfg->pos_X_offset[idx], &arc);

		data->angle_to_center[idx] = -RAD2DEG(arc) + 90.0f;
		arm_sqrt_f32(cfg->pos_X_offset[idx] * cfg->pos_X_offset[idx] +
				     cfg->pos_Y_offset[idx] * cfg->pos_Y_offset[idx],
			     &data->distance_to_center[idx]);
		idx++;
	}
	data->currTime = k_uptime_get_32();
	data->prevTime = data->currTime;
	return 0;
}

void cchassis_set_angle(const struct device *dev, float angle)
{
	chassis_data_t *data = dev->data;
	data->targetYaw = angle;
	data->angleControl = true;
}

void cchassis_set_speed(const struct device *dev, float x_speed, float y_speed)
{
	chassis_data_t *data = dev->data;
	data->targetXSpeed = x_speed;
	data->targetYSpeed = y_speed;
}

void cchassis_set_gyro(const struct device *dev, float gyro)
{
	chassis_data_t *data = dev->data;
	data->targetGyro = gyro;
	data->angleControl = false;
}

chassis_status_t *cchassis_get_status(const struct device *dev)
{
	chassis_data_t *data = dev->data;
	return &data->chassis_status;
}

void cchassis_resolve(chassis_data_t *data, const chassis_cfg_t *cfg)
{
	// data->targetRollSpeed = 0;
	float error = 0;
	if (!isnan(data->chassis_sensor_data.Yaw)) {
		float delta_Yaw = data->chassis_sensor_data.Yaw - data->targetYaw;
		delta_Yaw = fmodf(delta_Yaw, 360.0f);
		if (delta_Yaw > 180) {
			delta_Yaw -= 360.0f;
		} else if (delta_Yaw < -180) {
			delta_Yaw += 360.0f;
		}
		if (fabsf(delta_Yaw) > 0.8f) {
			error = delta_Yaw;
		}
	}

	int idx = 0;
	float currentSpeedX[CHASSIS_WHEEL_COUNT] = {0};
	float currentSpeedY[CHASSIS_WHEEL_COUNT] = {0};
	float currentGyro = 0;
	while (cfg->wheels[idx] != NULL) {
		// 由于我们的轮电机使用的是速度环PID控制
		// 我不认为它们会疯转，但可能存在遇到障碍
		wheel_status_t *status = wheel_get_speed(cfg->wheels[idx]);
		float angle = status->angle;
		float speed = status->speed;

		float sin_theta, cos_theta;

		arm_sin_cos_f32(data->angle_to_center[idx] + data->chassis_sensor_data.Yaw,
				&sin_theta, &cos_theta);
		currentSpeedX[idx] = speed * cos_theta;
		currentSpeedY[idx] = speed * sin_theta;

		idx++;
	}

	float delta_speed_X = data->targetXSpeed - data->currentXSpeed;
	float delta_speed_Y = data->targetYSpeed - data->currentYSpeed;
	float delta_speed = sqrtf(delta_speed_X * delta_speed_X + delta_speed_Y * delta_speed_Y);
	float delta_speed_angle = atan2f(delta_speed_Y, delta_speed_X);
	delta_speed = delta_speed > cfg->max_lin_accel    ? cfg->max_lin_accel
		      : delta_speed < -cfg->max_lin_accel ? -cfg->max_lin_accel
							  : delta_speed;
	data->currentXSpeed += delta_speed * cosf(delta_speed_angle);
	data->currentYSpeed += delta_speed * sinf(delta_speed_angle);
	float currentSpeed = sqrtf(data->currentXSpeed * data->currentXSpeed +
				   data->currentYSpeed * data->currentYSpeed);

	if (!isnan(data->chassis_sensor_data.Yaw)) {
		if (!data->angleControl) {
			data->currentGyro = data->targetGyro;
		} else {
			data->currentGyro =
				pid_calc_in(cfg->angle_pid, error,
					    k_cyc_to_us_near32(data->currTime - data->prevTime));
		}
	}
	data->currentGyro = data->currentGyro > cfg->max_gyro    ? cfg->max_gyro
			    : data->currentGyro < -cfg->max_gyro ? -cfg->max_gyro
								 : data->currentGyro;

	idx = 0;
	while (cfg->wheels[idx] != NULL) {
		float rollSpeedX = cfg->pos_Y_offset[idx] * data->currentGyro;
		float rollSpeedY = -cfg->pos_X_offset[idx] * data->currentGyro;
		float speedX, speedY;
		if (!data->track_angle) {
			speedX = rollSpeedX + data->currentXSpeed;
			speedY = rollSpeedY + data->currentYSpeed;
		} else {
			// 将角度转换为弧度
			float radians = -data->chassis_sensor_data.Yaw * PI / 180.0f;
			// 计算旋转后的新速度
			speedX = data->currentXSpeed * cosf(radians) -
				 data->currentYSpeed * sinf(radians) + rollSpeedX;
			speedY = data->currentXSpeed * sinf(radians) +
				 data->currentYSpeed * cosf(radians) + rollSpeedY;
		}
		float steerwheel_speed;
		arm_sqrt_f32(speedX * speedX + speedY * speedY, &steerwheel_speed);
		float steerwheel_angle = 0;
		arm_atan2_f32(speedY, speedX, &steerwheel_angle);

		steerwheel_angle = -RAD2DEG(steerwheel_angle) + 90.0f;

		if (fabsf(steerwheel_speed) > 0.1f || fabsf(data->currentGyro) > 0.15f) {
			wheel_set_speed(cfg->wheels[idx], steerwheel_speed, steerwheel_angle);
		} else {
			int err = wheel_set_static(cfg->wheels[idx],
						   data->angle_to_center[idx] + 90.0f);
			if (err < 0) {
				wheel_set_speed(cfg->wheels[idx], 0,
						data->angle_to_center[idx] + 90.0f);
			}
		}

		idx++;
	}
}

void cchassis_main_thread(const struct device *dev, void *ptr2, void *ptr3)
{
	ARG_UNUSED(ptr2);
	ARG_UNUSED(ptr3);

	chassis_data_t *data = dev->data;
	const chassis_cfg_t *cfg = dev->config;

	for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
		if (cfg->wheels[i] == NULL) {
			break;
		}
		wheel_set_static(cfg->wheels[i], 90.0f);
	}

	while (true) {
		k_msleep(1);
		while (!data->enabled) {
			k_msleep(100);
		}

		// printk("Yaw: %f, targetYaw: %f, delta_Yaw: %f\n", data->currentYaw,
		// data->targetYaw,
		//        delta_Yaw);
		if ((data->chassis_sensor_data.accel[2] - 9.8f) > 0.4f) {
			// We are in the air
		}
		cchassis_resolve(data, cfg);
	}
}

struct chassis_driver_api chassis_driver_api = {
	.set_angle = cchassis_set_angle,
	.set_speed = cchassis_set_speed,
	.set_gyro = cchassis_set_gyro,
	.get_status = cchassis_get_status,
};

K_THREAD_DEFINE(chassis_thread, CHASSIS_STACK_SIZE, cchassis_main_thread,
		DEVICE_DT_GET(DT_INST(0, DT_DRV_COMPAT)), NULL, NULL, 1, 0, 0);

DT_INST_FOREACH_STATUS_OKAY(CHASSIS_INIT)