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
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "vofa/justfloat.c"
#include "ekf/QuaternionEKF.c"
#include <math.h>

#undef PI
#define PI 3.14159265f

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define UART_NODE DT_NODELABEL(usart6)
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

#define ACCEL_NODE DT_NODELABEL(bmi08x_accel)
const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

#define GYRO_NODE DT_NODELABEL(bmi08x_gyro)
const struct device *gyro_dev = DEVICE_DT_GET(GYRO_NODE);

struct JFData data;

int main(void) {
    jf_send_init(uart_dev, &data, 100);

    // motor_control(motor1, ENABLE_MOTOR);
    // motor_set_mode(motor1, MIT);
    k_sleep(K_MSEC(50));

    int t = 0;
    while (1) {
        data.fdata[0] = sinf(PI * t / 1000.0f);
        data.fdata[1] = cosf(PI * t / 1000.0f);
        data.fdata[2] = sinf(PI * t / 2000.0f);
        data.fdata[3] = cosf(PI * t / 2000.0f);
        k_msleep(50);
        t += 50;
    }
}
