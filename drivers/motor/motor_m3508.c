/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/kernel/thread.h"
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include "motor_dji.h"

#define DT_DRV_COMPAT dji_m3508


static const struct motor_driver_api motor_api_funcs = {
    .motor_get_speed = dji_get_speed,
    .motor_get_torque = dji_get_torque,
    .motor_get_angle = dji_get_angle,
    .motor_set_speed = dji_set_speed,
    .motor_set_torque = dji_set_torque,
    .motor_set_angle = dji_set_angle,
};

#define DT_DRIVER_GET_CANPHY(inst) DT_GET_CANPHY(DT_DRIVER_GET_CANBUS_IDT(inst))
#define DT_DRIVER_INST_GET_MOTOR_IDT(inst) DT_DRV_INST(inst)
#define DT_DRIVER_INST_GET_CANBUS_IDT(inst) \
    DT_PHANDLE(DT_DRIVER_INST_GET_MOTOR_IDT(inst), can_channel)
#define DT_DRIVER_GET_CANBUS_ID(inst) \
    DT_NODE_CHILD_IDX(DT_DRIVER_INST_GET_CANBUS_IDT(inst))

extern const struct device *can_devices[];
extern const struct device *motor_devices[];

#define DMOTOR_DATA_INST(inst) \
static struct dji_motor_data dji_motor_data_##inst = { \
    .common = MOTOR_DT_DRIVER_DATA_INST_GET(inst), \
    .canbus_id = DT_DRIVER_GET_CANBUS_ID(inst), \
    .ctrl_struct = &motor_cans[DT_DRIVER_GET_CANBUS_ID(inst)], \
};

#define DMOTOR_CONFIG_INST(inst) \
static const struct dji_motor_config dji_motor_cfg_##inst = { \
    .common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst), \
    .gear_ratio = (float) DT_PROP(DT_DRV_INST(inst), gear_ratio) / 100.0f, \
};

#define DMOTOR_DEFINE_INST(inst) \
    MOTOR_DEVICE_DT_INST_DEFINE(inst, dji_init, NULL, \
                    &dji_motor_data_##inst, &dji_motor_cfg_##inst, \
                    POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
                    &motor_api_funcs);

#define DMOTOR_INST(inst) \
DMOTOR_CONFIG_INST(inst) \
DMOTOR_DATA_INST(inst) \
DMOTOR_DEFINE_INST(inst) 

DT_INST_FOREACH_STATUS_OKAY(DMOTOR_INST)