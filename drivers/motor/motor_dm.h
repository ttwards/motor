/*
 * motor_dm.h
 *
 * Header file for motor_dm.c
 */

#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "zephyr/kernel.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/can.h>

#define DT_DRV_COMPAT dm_motor

#define INT12_MAX 0x7FF
#define INT12_MIN -0x800

#define PI 3.14159265f

#define RAD2ROUND 1.0f / (2 * PI)
#define RAD2DEG   180.0f / PI

static const uint8_t enable_frame[]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t disable_frame[]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static const uint8_t set_zero_frame[]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static const uint8_t clear_error_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

#define CAN_SEND_STACK_SIZE 2048
#define CAN_SEND_PRIORITY   -1

#define SIZE_OF_ARRAY(x) (sizeof(x) / sizeof(x[0]))

struct dm_motor_data {
    struct motor_driver_data common;

    int tx_offset;

    // Control status
    bool online;

    int8_t missed_times;
    int8_t err;

    // Process round
    float delta_deg_sum; // Will be % by 360

    // Target
    float target_angle;
    float target_rpm;
    float target_torque;

    struct pid_single_config params;
};

struct dm_motor_config {
    struct motor_driver_config common;

    float gear_ratio;

    float v_max;
    float p_max;
    float t_max;
};

struct tx_frame {
    const struct device *can_dev;
    struct k_sem        *sem;
    struct can_frame     frame;
};

// 函数声明
void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data);

int   dm_motor_set_speed(const struct device *dev, float speed_rpm);
int   dm_motor_set_angle(const struct device *dev, float angle);
int   dm_motor_set_torque(const struct device *dev, float torque);
int   dm_motor_set_mode(const struct device *dev, enum motor_mode mode);
void  dm_motor_control(const struct device *dev, enum motor_cmd cmd);
float dm_motor_get_angle(const struct device *dev);
float dm_motor_get_speed(const struct device *dev);
float dm_motor_get_torque(const struct device *dev);

static const struct motor_driver_api motor_api_funcs = {
    .motor_get_speed  = dm_motor_get_speed,
    .motor_get_torque = dm_motor_get_torque,
    .motor_get_angle  = dm_motor_get_angle,
    .motor_set_speed  = dm_motor_set_speed,
    .motor_set_torque = dm_motor_set_torque,
    .motor_set_angle  = dm_motor_set_angle,
    .motor_control    = dm_motor_control,
    .motor_set_mode   = dm_motor_set_mode,
};

static struct k_sem tx_queue_sem[CAN_COUNT];

#define MOTOR_COUNT            DT_NUM_INST_STATUS_OKAY(dm_motor)
#define DM_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(DM_MOTOR_POINTER)};

#define CAN_BUS_PATH DT_PATH(canbus)

#define CAN_DEVICE_POINTER(node_id) DEVICE_DT_GET(DT_PROP(node_id, can_device))
static const struct device *can_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CAN_DEVICE_POINTER, (, ))};

static void dm_motor_ctrl_entry(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(dm_motor_ctrl_thread, CAN_SEND_STACK_SIZE, dm_motor_ctrl_entry, NULL, NULL, NULL,
                CAN_SEND_PRIORITY, 0, 10);
CAN_MSGQ_DEFINE(dm_can_rx_msgq, 12);
K_MSGQ_DEFINE(dm_can_tx_msgq, sizeof(struct tx_frame), MOTOR_COUNT, 4);

#define DMMOTOR_DATA_INST(inst)                                                                  \
    static struct dm_motor_data dm_motor_data_##inst = {                                         \
        .common        = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                    \
        .tx_offset     = 0,                                                                      \
        .online        = false,                                                                  \
        .missed_times  = 0,                                                                      \
        .err           = 0,                                                                      \
        .delta_deg_sum = 0,                                                                      \
        .target_angle  = 0,                                                                      \
        .target_rpm    = 0,                                                                      \
        .target_torque = 0,                                                                      \
        .params        = {0, 0},                                                                 \
    };

#define DMMOTOR_CONFIG_INST(inst)                                                                \
    static const struct dm_motor_config dm_motor_cfg_##inst = {                                  \
        .common     = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                     \
        .gear_ratio = (float)DT_PROP(DT_DRV_INST(inst), gear_ratio) / 100.0f,                    \
        .v_max      = (float)DT_PROP(DT_DRV_INST(inst), v_max) / 10.0f,                          \
        .p_max      = (float)DT_PROP(DT_DRV_INST(inst), p_max) / 10.0f,                          \
        .t_max      = (float)DT_PROP(DT_DRV_INST(inst), t_max) / 10.0f,                          \
    };

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)        \
    DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                   \
    MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define DMMOTOR_DEFINE_INST(inst)                                                                \
    MOTOR_DEVICE_DT_INST_DEFINE(inst, dm_init, NULL, &dm_motor_data_##inst,                      \
                                &dm_motor_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,   \
                                &motor_api_funcs);

#define DMMOTOR_INST(inst)                                                                       \
    DMMOTOR_CONFIG_INST(inst)                                                                    \
    DMMOTOR_DATA_INST(inst)                                                                      \
    DMMOTOR_DEFINE_INST(inst)

#endif // MOTOR_dm_H