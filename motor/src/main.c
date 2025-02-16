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
#include <ares/board/init.c>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define CAN1_NODE   DT_INST(0, vnd_canbus)
#define MOTOR1_NODE DT_INST(0, dm_motor)

#define FEEDBACK_STACK_SIZE 1536

const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);

/* CAN Feedback to console*/
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	for (int i = 0; i < 10000; i++) {
		motor_set_speed(motor1, i);
		LOG_INF("Speed: %.2f, Set: %.2f", (double)motor_get_speed(motor1), (double)i);
		k_msleep(20);
	}
}
K_THREAD_DEFINE(feedback_thread, FEEDBACK_STACK_SIZE, console_feedback, NULL, NULL, NULL, 1, 0,
		100);

int main(void)
{
	board_init();

	k_msleep(1000);
	motor_control(motor1, ENABLE_MOTOR);
	motor_set_mode(motor1, VO);
	while (1) {
		k_msleep(1500);
	}
}
