/*
 * Copyright (c) 2021-2022 Henrik Brix Andersen <henrik@brixandersen.dk>
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

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define CAN1_NODE   DT_INST(0, vnd_canbus)
#define MOTOR1_NODE DT_INST(0, dm_motor)

const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1  = DEVICE_DT_GET(MOTOR1_NODE);

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {
    while (1) {

        LOG_INF("rpm: motor1: %.2f\n", (double)motor_get_speed(motor1));
        k_msleep(500);
    }
}

int main(void) {
    // motor_control(motor1, ENABLE_MOTOR);
    // motor_set_mode(motor1, MIT);
    k_sleep(K_MSEC(50));
    // motor_set_speed(motor1, 10);
    motor_set_mode(motor1, MIT);
    motor_control(motor1, ENABLE_MOTOR);
    motor_set_speed(motor1, 2);

    /* Start Feedback thread*/
    struct k_thread feedback_thread_data;
    k_thread_create(&feedback_thread_data,
                    feedback_stack_area, // 修改为 can_send_stack_area
                    K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback, (void *)motor1,
                    NULL, NULL, 0, 0, K_MSEC(300));

    while (1) {
        // motor1_rpm = motor_get_speed(motor1);
        // LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm, (double)motor_get_speed(motor1));
        k_msleep(500);
        // k_sleep(K_FOREVER);
    }
}
