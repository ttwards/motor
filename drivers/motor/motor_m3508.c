/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT dji_m3508

#include <zephyr/logging/log.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>


LOG_MODULE_REGISTER(motor_m3508, CONFIG_MOTOR_LOG_LEVEL);

#define CAN_DEVICE_NAME DT_LABEL(DT_CHOSEN(zephyr_canbus))
#define CAN_MSG_ID 0x123
#define CAN_MSG_DLC 8
#define SEND_INTERVAL K_SECONDS(1)

K_THREAD_STACK_DEFINE(can_stack_area, 1024);
struct k_thread can_thread_data;

const struct device *can_dev;

struct motor_info {
    const char *label;
    uint32_t current;
    uint32_t torque;
    uint32_t rpm;
};

#define MOTOR_INFO_INIT(node_id) { \
    .label = DT_PROP(node_id, label), \
    .current = DT_PROP(node_id, current), \
    .torque = DT_PROP(node_id, torque), \
    .rpm = DT_PROP(node_id, rpm) \
}

#define MOTOR_NODE_ID(node_id) DT_CHILD(DT_PATH(motor), node_id)

#define MOTOR_INFO_FOREACH(node_id) \
    MOTOR_INFO_INIT(MOTOR_NODE_ID(node_id)),

struct motor_info motors[] = {
    DT_FOREACH_CHILD(DT_PATH(motor), MOTOR_INFO_FOREACH)
};



static int motor_m3508_start(const struct device *dev) {
    can_dev = device_get_binding(CAN_DEVICE_NAME); 
        LOG_ERR("CAN: Device driver not found.");
        return -ENODEV;
    }

    k_thread_create(&can_thread_data, can_stack_area,
                    K_THREAD_STACK_SIZEOF(can_stack_area),
                    can_send_thread,
                    NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);

    return 0;
}

DEVICE_DT_INST_DEFINE(0, motor_m3508_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);