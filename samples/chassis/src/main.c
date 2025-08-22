/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include "devices.h"
#include <zephyr/drivers/chassis.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* CAN Feedback to console*/
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	float angle = 0;
	int n = 0;
	while (1) {
		float X = sbus_get_percent(sbus, 3);
		float Y = sbus_get_percent(sbus, 1);

		chassis_set_speed(chassis, X * 7, Y * 7);
		chassis_set_gyro(chassis, -sbus_get_percent(sbus, 0) * 2.0f);

		k_msleep(20);
	}
}
K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 1, 0, 100);

int pub_cnt = 0;

int main(void)
{

	k_sleep(K_MSEC(50));
	chassis_set_enabled(chassis, false);
	IMU_Sensor_trig_init(accel_dev, gyro_dev);
	chassis_set_enabled(chassis, true);

	IMU_Sensor_set_update_cb(Sensor_update_cb);

	chassis_set_angle(chassis, 0);

	while (1) {
		k_sleep(K_MSEC(500));
	}
}
