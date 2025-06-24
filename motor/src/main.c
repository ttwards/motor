/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include <ares/board/init.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define CAN1_NODE   DT_INST(0, vnd_canbus)
#define MOTOR1_NODE DT_INST(0, dji_motor)
#define MOTOR2_NODE DT_INST(1, dji_motor)

#define FEEDBACK_STACK_SIZE 1536

const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);

/* CAN Feedback to console*/
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	motor_status_t status;
	for (int i = 0; i < 10000; i++) {
		motor_get(motor1, &status);
		LOG_INF("Motor 1: Speed: %.2f, Set: %.2f", (double)status.rpm, (double)i);
		motor_get(motor2, &status);
		LOG_INF("Motor 2: Speed: %.2f, Set: %.2f", (double)status.rpm, (double)i);
		k_msleep(5);
	}
}
K_THREAD_DEFINE(feedback_thread, FEEDBACK_STACK_SIZE, console_feedback, NULL, NULL, NULL, 1, 0,
		100);

int main(void)
{
	k_msleep(1000);
	motor_control(motor1, ENABLE_MOTOR);
	motor_control(motor2, ENABLE_MOTOR);
	while (1) {
		k_msleep(1500);
	}
}
