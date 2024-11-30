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
#include <zephyr/drivers/sbus.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define MOTOR1_NODE DT_INST(0, dji_motor)
#define MOTOR2_NODE DT_INST(1, dji_motor)
#define MOTOR3_NODE DT_INST(2, dji_motor)

#define SBUS_NODE DT_NODELABEL(sbus0)

#define CPU_NODE DT_NODELABEL(cpu0)

const struct device *cpu_dev = DEVICE_DT_GET(CPU_NODE);
const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1  = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2  = DEVICE_DT_GET(MOTOR2_NODE);
const struct device *motor3  = DEVICE_DT_GET(MOTOR3_NODE);
const struct device *sbus    = DEVICE_DT_GET(SBUS_NODE);

k_tid_t feedback_tid = 0;

float motor1_rpm = 0;
float motor2_rpm = 0;
float motor3_rpm = 0;

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {
    while (1) {
        LOG_INF("rpm: motor1: %.2f	%.2f\n", (double)motor1_rpm,
                (double)motor_get_speed(motor1));
        LOG_INF("rpm: motor2: %.2f %.2f\n", (double)motor2_rpm, (double)motor_get_speed(motor2));
        LOG_INF("rpm: motor3: %.2f %.2f\n", (double)motor3_rpm, (double)motor_get_speed(motor3));
        k_msleep(500);
    }
}
// 1 |
// 2 /
// 3 -
// 1  2  3
//       -
//
// |     /
/*
v_3 = (1/sqrt(2)) * (v_x - v_Y)
*/
static void chassis_calc(float vert_x, float vert_y) {
    float vert_x_rpm = vert_x * 120;
    float vert_y_rpm = vert_y * 120;
    motor1_rpm       = vert_x_rpm;
    motor3_rpm       = vert_y_rpm;
    motor2_rpm       = (-vert_x_rpm - vert_y_rpm) * 0.707f;
    motor_set_speed(motor1, motor1_rpm);
    motor_set_speed(motor2, motor2_rpm);
    motor_set_speed(motor3, motor3_rpm);
}

int main(void) {

    motor_set_zero(motor1);
    motor_set_zero(motor2);
    motor_set_zero(motor3);

    motor_set_angle(motor1, 0);
    motor_set_angle(motor2, 0);
    motor_set_angle(motor3, 0);

    /* Start Feedback thread*/
    struct k_thread feedback_thread_data;
    feedback_tid = k_thread_create(&feedback_thread_data,
                                   feedback_stack_area, // 修改为 can_send_stack_area
                                   K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
                                   (void *)motor1, NULL, NULL, 0, 0, K_MSEC(300));
    while (1) {
        chassis_calc(sbus_get_percent(sbus, 1), sbus_get_percent(sbus, 3));
        k_msleep(10);
    }
}
