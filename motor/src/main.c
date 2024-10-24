/*
 * Copyright (c) 2021-2022 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/motor.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define MOTOR1_NODE DT_INST(0, dji_m3508)
#define MOTOR2_NODE DT_INST(1, dji_m3508)

#define CPU_NODE DT_NODELABEL(cpu0)

const struct device *cpu_dev = DEVICE_DT_GET(CPU_NODE);
const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);

k_tid_t feedback_tid = 0;

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {
	while (1) {
		LOG_INF("rpm: motor1:%d\tmotor2:%d\n", motor_get_speed(motor1), motor_get_speed(motor2));
		k_msleep(280);
	}
}

int main(void)
{
	// int err = 0;
	// // err = device_is_ready(motor_dev);
	// /* CAN Device init.*/
	// err = device_is_ready(can_dev);	
	// if (err != 0)
	// 	printk("CAN device not ready");
	// // for(;;) printk("Fuck USART!");
	// err = can_start(can_dev);
	// if (err != 0)
	// 	printk("Error starting CAN controller (err %d)", err);
	motor_set_speed(motor1, 1222);
	motor_set_speed(motor2, 1222);

	/* Start Feedback thread*/
	struct k_thread feedback_thread_data;
	feedback_tid = k_thread_create(&feedback_thread_data, feedback_stack_area, // 修改为 can_send_stack_area
                                K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
								motor1, NULL, NULL,
								2, 0, K_MSEC(300));
	while (1) {
		// motor_set_speed(motor1, -1222);
		// motor_set_speed(motor2, -1222);
        // k_sleep(K_MSEC(200));
		// motor_set_speed(motor1, 1222);
		// motor_set_speed(motor2, 1222);
        k_sleep(K_MSEC(200));
    }
}
