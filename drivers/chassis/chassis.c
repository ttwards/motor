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
#include <zephyr/zbus/zbus.h>
#include "zephyr/kernel.h"
#include "arm_math.h"
#include <math.h>

#define DT_DRV_COMPAT ares_chassis

#define RAD2DEG(x) 57.2957795131f * x

LOG_MODULE_REGISTER(chassis, CONFIG_MOTOR_LOG_LEVEL);

ZBUS_MSG_SUBSCRIBER_DEFINE(chassis_sensor_msg_suscriber);

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
	pid_reg_input(cfg->angle_pid, &(data->pid_input), &(data->static0));
	pid_reg_time(cfg->angle_pid, &(data->currTime), &(data->prevTime));
	pid_reg_output(cfg->angle_pid, &(data->targetGyro));
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
	if (!float_equal(gyro, 0)) {
		data->targetGyro = gyro;
		data->angleControl = false;
	} else if (!data->angleControl) {
		data->targetYaw = data->currentYaw;
		data->angleControl = true;
	}
}

chassis_status_t *cchassis_get_status(const struct device *dev)
{
	chassis_data_t *data = dev->data;
	return &data->chassis_status;
}

void cchassis_resolve(chassis_data_t *data, const chassis_cfg_t *cfg)
{
	data->prevTime = data->currTime;
	data->currTime = k_cycle_get_32();

	if (data->angleControl) {
		pid_calc(cfg->angle_pid);
	}
	// data->targetRollSpeed = 0;

	int idx = 0;
	float currentSpeedX[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT] = {0};
	float currentSpeedY[CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT] = {0};
	float currentGyro = 0;
	while (cfg->wheels[idx] != NULL) {
		// 由于我们的轮电机使用的是速度环PID控制
		// 我不认为它们会疯转，但可能存在遇到障碍
		wheel_status_t *status = wheel_get_speed(cfg->wheels[idx]);
		float angle = status->angle;
		float speed = status->speed;

		float sin_theta, cos_theta;

		arm_sin_cos_f32(data->angle_to_center[idx] + data->currentYaw, &sin_theta,
				&cos_theta);
		currentSpeedX[idx] = speed * cos_theta;
		currentSpeedY[idx] = speed * sin_theta;

		idx++;
	}

	idx = 0;
	while (cfg->wheels[idx] != NULL) {
		float rollSpeedX = cfg->pos_Y_offset[idx] * data->targetGyro;
		float rollSpeedY = -cfg->pos_X_offset[idx] * data->targetGyro;
#if TRACK_ANGLE
		float speedX = rollSpeedX + data->targetXSpeed;
		float speedY = rollSpeedY + data->targetYSpeed;
#else
		// 将角度转换为弧度
		float radians = -data->currentYaw * PI / 180.0f;

		// 计算旋转后的新速度
		float speedX = data->targetXSpeed * cosf(radians) -
			       data->targetYSpeed * sinf(radians) + rollSpeedX;
		float speedY = data->targetXSpeed * sinf(radians) +
			       data->targetYSpeed * cosf(radians) + rollSpeedY;
#endif
		float steerwheel_speed;
		arm_sqrt_f32(speedX * speedX + speedY * speedY, &steerwheel_speed);
		float steerwheel_angle = 0;
		arm_atan2_f32(speedY, speedX, &steerwheel_angle);

		steerwheel_angle = -RAD2DEG(steerwheel_angle) + 90.0f;

		if (fabsf(steerwheel_speed) > 0.1f || fabsf(data->targetGyro) > 0.15f) {
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
	const struct zbus_channel *chan;

	chassis_data_t *data = dev->data;
	const chassis_cfg_t *cfg = dev->config;

	for (int i = 0; i < CONFIG_CHASSIS_MAX_STEERWHHEL_COUNT; i++) {
		if (cfg->wheels[i] == NULL) {
			break;
		}
		wheel_set_static(cfg->wheels[i], 90.0f);
	}

	k_msleep(1000);

	struct pos_data pos = {0};

	while (true) {
		if (data->chassis_sensor_zbus != NULL) {
			zbus_sub_wait_msg(&chassis_sensor_msg_suscriber, &chan, &pos, K_MSEC(1));
			if (data->chassis_sensor_zbus != chan) {
				continue;
			}
		} else {
			k_msleep(1);
		}
		data->currentYaw = pos.Yaw;

		float delta_Yaw = data->targetYaw - data->currentYaw;
		delta_Yaw = fmodf(delta_Yaw, 360.0f);
		if (delta_Yaw > 180) {
			delta_Yaw -= 360.0f;
		} else if (delta_Yaw < -180) {
			delta_Yaw += 360.0f;
		}
		if (fabsf(delta_Yaw) > 0.8f) {
			data->pid_input = delta_Yaw;
		} else {
			data->pid_input = 0;
		}

		printk("Yaw: %f, targetYaw: %f, delta_Yaw: %f\n", data->currentYaw, data->targetYaw,
		       delta_Yaw);
		if ((pos.accel[2] - 9.8f) > 0.4f) {
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