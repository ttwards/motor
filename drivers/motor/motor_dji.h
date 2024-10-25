/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/logging/log.h"
#include "zephyr/sys/time_units.h"
#include <stdbool.h>
#include <sys/types.h>
#include <zephyr/kernel/thread.h>
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <limits.h>

LOG_MODULE_REGISTER(motor_dji, CONFIG_MOTOR_LOG_LEVEL);


#define CAN_DEVICE_NAME DT_LABEL(DT_CHOSEN(zephyr_canbus))
#define CAN_MSG_DLC 8
#define SEND_INTERVAL K_MSECONDS(10)

#define CAN_SEND_STACK_SIZE 4096
#define CAN_SEND_PRIORITY -1

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/** @brief Macros for each in dts */
#define MOTOR_NODE DT_NODELABEL(motor)
#define CAN_BUS_NODE DT_NODELABEL(canbus)

#define MOTOR_PATH DT_PATH(motor)
#define CAN_BUS_PATH DT_PATH(canbus)

#define IS_DJI_COMPAT(node_id) \
    (DT_NODE_HAS_COMPAT(node_id, dji_m3508) || \
     DT_NODE_HAS_COMPAT(node_id, dji_m2006) || \
     DT_NODE_HAS_COMPAT(node_id, dji_m6020) || \
     DT_NODE_HAS_COMPAT(node_id, dji_others)) // Add other DJI compatible strings as needed

#define DJI_DEVICE_POINTER(node_id) DEVICE_DT_GET(node_id)
#define CAN_DEVICE_POINTER(node_id) DEVICE_DT_GET(DT_PROP(node_id, can_device))

const struct device *motor_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(MOTOR_PATH, DJI_DEVICE_POINTER, (,))
};

const struct device *can_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CAN_DEVICE_POINTER, (,))
};

#define GET_CANPHY_POINTER_BY_IDX(node_id, idx) \
    DEVICE_DT_GET(DT_PHANDLE(DT_CHILD_BY_IDX(node_id, idx)))
#define CANPHY_BY_IDX(idx) GET_CANPHY_POINTER_BY_IDX(CAN_BUS_PATH, idx)

#define MOTOR_COUNT sizeof(motor_devices)/sizeof(motor_devices[0])
#define CAN_COUNT DT_NUM_INST_STATUS_OKAY(vnd_canbus)

#define GET_CAN_CHANNEL_IDT(node_id) DT_PHANDLE(node_id, can_channel)
#define GET_CAN_DEV(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_device))
#define GET_TX_ID(node_id) DT_PHANDLE(node_id, tx_addr)
#define GET_RX_ID(node_id) DT_PHANDLE(node_id, rx_addr)
#define GET_CTRL_ID(node_id) GET_TX_ID(node_id) & 0xF

#define CTRL_STRUCT_DATA(node_id) \
{ \
    .can_dev = DT_GET_CANPHY_BY_BUS(node_id), \
    .motor_rpm = {0}, \
    .flags = 0, \
    .info = {0}, \
    .enabled[1] = false, \
    .enabled[0] = false, \
    .mask = 0, \
}

// extern k_tid_t dji_motor_ctrl_thread;

/*  canbus_id_t specifies the ID of the CAN bus the motor is on
    which is defined in motor_devices[] */
typedef uint8_t canbus_id_t;

/*  allmotor_id_t specifies the ID of the motor
    which is in the flags
    use flags |= 1 << all_motor_id*/
typedef uint8_t allmotor_id_t;

/*  can_id_t specifies the ID of the motor in motor_controller struct
    find the rpm in motor_cans->motor_rpm[can_id] */
typedef uint8_t can_id_t;

struct motor_info {
    uint16_t angle;
    int16_t current;
    int16_t torque;
    int16_t rpm;
    int8_t temperature;
    int64_t t_delta_rpm;
    int16_t last_rpm;
    uint64_t last_time;
    uint64_t lastlast_time;
};
struct motor_controller {
    const struct device *can_dev;
    int16_t motor_rpm[8];
    int16_t current[8];
    uint8_t flags;
    uint8_t mask;
    const struct device *motor_devs[8];
    bool enabled[2];
    struct motor_info info[8];
};

struct dji_motor_data {
    struct motor_driver_data common;
    canbus_id_t canbus_id;
    struct motor_controller *ctrl_struct;
};

struct dji_motor_config {
    struct motor_driver_config common;
};

static void can_send_entry(struct motor_controller *ctrl_struct, void *arg2, void *arg3);
struct motor_controller motor_cans[CAN_COUNT] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CTRL_STRUCT_DATA, (,))
};


#ifndef MDJI_FIRST
#define MDJI_FIRST
K_THREAD_DEFINE(dji_motor_ctrl_thread, CAN_SEND_STACK_SIZE, \
    can_send_entry, motor_cans, can_devices, motor_devices, \
    CAN_SEND_PRIORITY, 0, 10);
#endif 

static inline can_id_t can_id(const struct device *dev) {
    const struct motor_driver_config *cfg = dev->config;
    return (cfg->rx_id & 0xF) - 1;
}

static inline can_id_t can_frame_id(const struct device *dev) {
    const struct motor_driver_config *cfg = dev->config;
    return (cfg->rx_id & 0xF) - 1 + ((cfg->tx_id - 0x200) << 2);
}

static inline allmotor_id_t all_motor_id(const struct device *dev) {
    const struct motor_driver_config *cfg = dev->config;
    struct dji_motor_data *data = dev->data;
    return (data->canbus_id << 3) + (cfg->rx_id & 0xF) - 1;
}

static inline int16_t int32_to_int16(int32_t value) {
    if (value > INT16_MAX) {
        return INT16_MAX;
    } else if (value < INT16_MIN) {
        return INT16_MIN;
    } else {
        return (int16_t)value;
    }
}

static int16_t pid_calc(const struct dji_motor_config *cfg, struct motor_controller *ctrl_struct, uint8_t canbus_id) {
    uint8_t id = (cfg->common.rx_id & 0xF) - 1;
    double kp = cfg->common.k_p;
    double ki = cfg->common.k_i;
    double kd = cfg->common.k_d;
    int16_t rpm = ctrl_struct->info[id].rpm;
    int16_t last_rpm = ctrl_struct->info[id].last_rpm;
    int16_t targetRPM = ctrl_struct->motor_rpm[id];
    double trpm = (double) (ctrl_struct->info[id].t_delta_rpm) / 1000000; 
    double de_rpm = (double) (rpm - last_rpm) / (k_us_to_ticks_near64(ctrl_struct->info[id].last_time - ctrl_struct->info[id].lastlast_time) * 1000000);
    int32_t et = targetRPM - rpm;
    int32_t ut = (int32_t)(kp * (et + ki * trpm + kd * de_rpm));
    return int32_to_int16(ut);
}


static int8_t dji_set_speed(const struct device *dev, int16_t speed_rpm) {
    struct dji_motor_data *data = dev->data;
    struct dji_motor_config *cfg = dev->config;
    uint8_t id = (cfg->common.rx_id & 0xF) - 1;
    struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
    ctrl_struct->motor_rpm[id] = speed_rpm;
    ctrl_struct->current[id] = pid_calc(ctrl_struct->motor_devs[id]->config, ctrl_struct, data->canbus_id);
    return 0;
}

static int16_t dji_get_speed(const struct device *dev) {
    struct dji_motor_data *data = dev->data;
    struct dji_motor_config *cfg = dev->config;
    struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
    int16_t rpm = ctrl_struct->info[(cfg->common.rx_id & 0xF) - 1].rpm;
    return rpm;
}

static int16_t dji_get_torque(const struct device *dev) {
    struct dji_motor_data *data = dev->data;
    struct dji_motor_config *cfg = dev->config;
    struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
    int16_t torque = ctrl_struct->info[(cfg->common.rx_id & 0xF) - 1].torque;
    return torque;
}

static int dji_init(const struct device *dev) {
/*
    Each motor find their own controller and write their own data, which cannot be done at the first init
    'Cause the other motors have not been instantiated.
 */
    if (dev) {
        const struct dji_motor_config *cfg = dev->config;
        struct dji_motor_data *data = (struct dji_motor_data *)dev->data;
        if(cfg->common.rx_id > 0x204 && data->ctrl_struct->enabled[1] == false) {
            data->ctrl_struct->mask |= 0xF0;
            data->ctrl_struct->enabled[1] = true;
        }
        else if(data->ctrl_struct->enabled[0] == false) {
            data->ctrl_struct->mask |= 0xF;
            data->ctrl_struct->enabled[0] = true;
        }
        uint8_t id = (cfg->common.rx_id & 0xF) - 1;
        data->ctrl_struct->motor_devs[id] = dev;
        data->ctrl_struct->info[id].last_time = 0;
        data->ctrl_struct->flags |= 1 << id;
        data->ctrl_struct->mask ^= 1 << id;
        if(!device_is_ready(cfg->common.phy)) return -1;
    }
    k_thread_start(dji_motor_ctrl_thread);
    return 0;
}



void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data) {
    uint64_t current_time = k_cycle_get_64();
    struct motor_controller *ctrl_struct = (struct motor_controller *)user_data;
    struct can_frame rx_frame = *frame;
    uint8_t id = (rx_frame.id & 0xF) - 1;
    uint8_t can_id = 0;
    for(int i = 0; i < CAN_COUNT; i++) {
        if(can_dev == ctrl_struct[i].can_dev) {
            can_id = i;
            break;
        }
    }
    uint64_t last_time = ctrl_struct->info[id].last_time;
    if(!ctrl_struct) return;
    struct motor_info *motor_info = &ctrl_struct[can_id].info[id];
    if(!motor_info) return;
	motor_info->angle = COMBINE_HL8(rx_frame.data[0], rx_frame.data[1]);
	motor_info->rpm = COMBINE_HL8(rx_frame.data[2], rx_frame.data[3]);
	motor_info->torque = COMBINE_HL8(rx_frame.data[4], rx_frame.data[5]);
	motor_info->temperature = rx_frame.data[6];
    int16_t delta = ctrl_struct->motor_rpm[id] - motor_info->rpm;
    uint64_t dtus = k_cyc_to_us_near64(current_time - last_time);
    int64_t drpm = dtus * delta;
    motor_info->t_delta_rpm += drpm;
	ctrl_struct[can_id].flags |= 1 << id;
    uint8_t high4 = (ctrl_struct[can_id].flags | ctrl_struct[can_id].mask) & 0xF0;
    uint8_t low4 = (ctrl_struct[can_id].flags | ctrl_struct[can_id].mask) & 0xF;
    ctrl_struct->info[id].last_time = current_time;
    ctrl_struct->info[id].lastlast_time = last_time;
    if(high4 == 0xF0) {
        ctrl_struct->flags ^= 0xF0;
        k_thread_resume(dji_motor_ctrl_thread);
        return;
    }
    else if(low4 == 0xF) {
        ctrl_struct->flags ^= 0xF;
        k_thread_resume(dji_motor_ctrl_thread);
        return;
    }
    
}

static const struct can_filter filter20x = {
	.id = 0x200,
	.mask = 0x3F0,
	.flags = 0
};

static inline void can_calc(struct can_frame *frame, uint16_t tx_id, int16_t *currents) {
    for(int i = 0; i < 2; i++){
        frame[i].id = tx_id - i;
        frame[i].dlc = 8;
        frame[i].flags = 0;
        uint8_t data[8] = {0};
        for(int j = 0; j < 4; j++) {
            data[j * 2] = HIGH_BYTE(currents[i * 4 + j]);
            data[j * 2 + 1] = LOW_BYTE(currents[i * 4 + j]);
        }
        memcpy(frame[i].data, data, sizeof(data));
    }
}

static void can_tx_callback(const struct device *can_dev, int error, void *user_data)
{
	struct k_sem *queue_sem = user_data;
	k_sem_give(queue_sem);
}

static struct k_sem tx_queue_sem;
struct can_frame txframe[CAN_COUNT][2];

static void can_send_entry(struct motor_controller *ctrl_struct, void *arg2, void *arg3) {
    k_sem_init(&tx_queue_sem, 24, 24); // 初始化信号量
    struct device *can_dev = NULL;
    for (int i = 0; i < CAN_COUNT; i++) {
        can_dev = (struct device*) ctrl_struct[i].can_dev;
        can_start(can_dev);
        int err = can_add_rx_filter(can_dev, can_rx_callback, &ctrl_struct[i], &filter20x);
        if (err != 0)
            LOG_ERR("Error adding CAN filter (err %d)", err);
        for (int j = 0; j < 2; j++) {
            can_calc(txframe[i], 0x200, ctrl_struct[i].current);
        }
    }
    int err = 0;

    k_sleep(K_MSEC(500));
    while (1) {
        for (int8_t i = 0; i < CAN_COUNT; i++) {
            for(int j = 0; j < 8; j++) {
                if ((ctrl_struct[i].mask & (1 << j)) == 0 && ctrl_struct[i].enabled[j > 3 ? 1 : 0]) {
                    ctrl_struct[i].current[j] = pid_calc(ctrl_struct[i].motor_devs[j]->config, &ctrl_struct[i], i);
                }
            }
            can_calc(txframe[i], 0x200, ctrl_struct[i].current);
            can_dev = (struct device*) ctrl_struct[i].can_dev;
            if(motor_cans[i].enabled[0]) {
                err = k_sem_take(&tx_queue_sem, K_NO_WAIT);
                if(err == 0)
                    err = can_send(can_dev, &txframe[i][0], K_NO_WAIT, can_tx_callback, &tx_queue_sem);
            }
            if(motor_cans[i].enabled[1]) {
                err = k_sem_take(&tx_queue_sem, K_NO_WAIT);
                if(err == 0)
                    err = can_send(can_dev, &txframe[i][1], K_NO_WAIT, can_tx_callback, &tx_queue_sem);
            }
            if (err != 0)
                LOG_ERR("Error sending CAN frame (err %d)", err);
        }
        k_thread_suspend(dji_motor_ctrl_thread);
    }
}

