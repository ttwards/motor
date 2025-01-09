/*	
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
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
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define G 9.80665f

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE    DT_CHOSEN(zephyr_canbus)
#define MOTOR1_NODE    DT_INST(0, dji_motor)
#define MOTOR2_NODE    DT_INST(1, dji_motor)
#define MOTOR3_NODE    DT_INST(2, dji_motor)
#define IMU_ACCEL_NODE DT_NODELABEL(bmi08x_accel)
#define IMU_GYRO_NODE  DT_NODELABEL(bmi08x_gyro)

#define SBUS_NODE DT_NODELABEL(sbus0)

#define CPU_NODE DT_NODELABEL(cpu0)

const struct device *cpu_dev = DEVICE_DT_GET(CPU_NODE);
const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
const struct device *motor1  = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2  = DEVICE_DT_GET(MOTOR2_NODE);
const struct device *motor3  = DEVICE_DT_GET(MOTOR3_NODE);
const struct device *sbus    = DEVICE_DT_GET(SBUS_NODE);
const struct device *accel   = DEVICE_DT_GET(IMU_ACCEL_NODE);
const struct device *gyro    = DEVICE_DT_GET(IMU_GYRO_NODE);

k_tid_t feedback_tid = 0;

float motor1_rpm = 0;
float motor2_rpm = 0;
float motor3_rpm = 0;

// 定义重力加速度常数
const float GRAVITY_EARTH = 9.80665f;
// 加速度计全量程（g）
const float accel_full_scale = 16.0f;
const float gyro_full_scale  = 2000.0f;
// 计算缩放因子
float accel_scale = (GRAVITY_EARTH * accel_full_scale) / 32768.0f;
float gyro_scale  = (gyro_full_scale) / 32768.0f;
#define BMI08X_TEMP_OFFSET 23

// 温度转换函数
static float convert_temp_bmi08x(const struct sensor_value *temp_data) {
    // 按照驱动中的计算方式：
    // temp_micro = BMI08X_TEMP_OFFSET * 1000000ULL + temp_raw * 31250ULL
    int32_t temp_micro = (int32_t)temp_data->val1 * 1000000 + temp_data->val2;

    // 转换为实际温度值
    float temp = (float)(temp_micro - BMI08X_TEMP_OFFSET * 1000000ULL) / 31250.0f;
    return temp + BMI08X_TEMP_OFFSET;
}

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {
    while (1) {
        LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm, (double)motor_get_speed(motor1));
        LOG_INF("rpm: motor2: %.2f %.2f\n", (double)motor2_rpm, (double)motor_get_speed(motor2));
        LOG_INF("rpm: motor3: %.2f %.2f\n", (double)motor3_rpm, (double)motor_get_speed(motor3));
        struct sensor_value accel_data[3];
        struct sensor_value gyro_data[3];
        struct sensor_value temp_data;
        sensor_sample_fetch(accel);
        sensor_sample_fetch(gyro);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, accel_data);
        sensor_channel_get(gyro, SENSOR_CHAN_GYRO_XYZ, gyro_data);
        sensor_channel_get(accel, SENSOR_CHAN_DIE_TEMP, &temp_data);
        float accel_x  = ((float)accel_data[0].val1 + (float)accel_data[0].val2 / 1000000.0f);
        float accel_y  = ((float)accel_data[1].val1 + (float)accel_data[1].val2 / 1000000.0f);
        float accel_z  = ((float)accel_data[2].val1 + (float)accel_data[2].val2 / 1000000.0f);
        float accel_c  = sqrtf(powf(accel_x, 2) + powf(accel_y, 2) + powf(accel_z, 2));
        float gyro_x   = (float)gyro_data[0].val1 + (float)gyro_data[0].val2 / 1000000.0f;
        float gyro_y   = (float)gyro_data[1].val1 + (float)gyro_data[1].val2 / 1000000.0f;
        float gyro_z   = (float)gyro_data[2].val1 + (float)gyro_data[2].val2 / 1000000.0f;
        float gryo_c   = sqrtf(powf(gyro_x, 2) + powf(gyro_y, 2) + powf(gyro_z, 2));
        float temp_raw = ((float)temp_data.val1 + (float)temp_data.val2 / 1000000.0f);
        LOG_INF("accel: x %.3f y %.3f z %.3f\n", (double)accel_x, (double)accel_y,
                (double)accel_z);
        LOG_INF("gyro: x %.3f y %.3f z %.3f\n", (double)gyro_x, (double)gyro_y, (double)gyro_z);
        LOG_INF("IMU Temp: %.3f\n", (double)temp_raw);
        // LOG_INF("TEMP MSB: %d LSB: %d\n", temp_data.val1, temp_data.val2);
        LOG_INF("Accel %.2f m/s^2\n", (double)accel_c);
        LOG_INF("Gyro %.2f\n", (double)gryo_c);

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
        k_msleep(20);
    }
}
