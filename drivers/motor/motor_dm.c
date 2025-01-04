/*	
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "motor_dm.h"
#include "syscalls/kernel.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/motor.h"
#include "zephyr/drivers/pid.h"
#include "zephyr/kernel.h"

#define DT_DRV_COMPAT dm_motor

LOG_MODULE_REGISTER(motor_dm, CONFIG_MOTOR_LOG_LEVEL);

/**
 * @brief Converts an unsigned integer to a float, given range and number of bits.
 *
 * This function takes an unsigned integer and maps it to a float value within
 * the specified range [x_min, x_max] based on the number of bits.
 *
 * @param x_int The unsigned integer value to be converted.
 * @param x_min The minimum value of the target float range.
 * @param x_max The maximum value of the target float range.
 * @param bits The number of bits representing the unsigned integer.
 * @return The corresponding float value within the range [x_min, x_max].
 */

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief Converts a float to an unsigned int, given range and number of bits.
 *
 * This function takes a floating-point number and converts it to an unsigned
 * integer representation based on the specified range and number of bits.
 *
 * @param x The floating-point number to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits for the unsigned integer representation.
 *
 * @return The unsigned integer representation of the floating-point number.
 */
static inline int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static int get_can_id(const struct device *dev) {
    const struct dm_motor_config *cfg = dev->config;
    for (int i = 0; i < CAN_COUNT; i++) {
        if (can_devices[i] == cfg->common.phy) {
            return i;
        }
    }
    return -1;
}

static void can_tx_callback(const struct device *can_dev, int error, void *user_data) {
    struct k_sem *queue_sem = user_data;
    if (!error)
        k_sem_give(queue_sem);
}

int dm_init(const struct device *dev) {
    const struct dm_motor_config *cfg    = dev->config;
    int                           can_id = get_can_id(dev);
    k_sem_init(&tx_queue_sem[can_id], 3, 3); // 初始化信号量
    if (!device_is_ready(cfg->common.phy))
        return -1;
    k_thread_start(dm_motor_ctrl_thread);
    return 0;
}

int dm_send_queued(struct tx_frame *frame, struct k_msgq *msgq) {
    int err = k_sem_take(frame->sem, K_NO_WAIT);
    if (err == 0) {
        err = can_send(frame->can_dev, &frame->frame, K_NO_WAIT, can_tx_callback, frame->sem);
        if (err) {
            LOG_ERR("TX queue full, will be put into msgq: %d", err);
        }
    } else if (err < 0) {
        // LOG_ERR("CAN hardware TX queue is full. (err %d)", err);
        err = k_msgq_put(msgq, frame, K_NO_WAIT);
        if (err) {
            LOG_ERR("Failed to put CAN frame into TX queue: %d", err);
        }
    }
    return err;
}

int dm_queue_proceed(struct k_msgq *msgq) {
    struct tx_frame frame;
    int             err     = 0;
    bool            give_up = false;
    while (!k_msgq_get(msgq, &frame, K_NO_WAIT)) {
        err = k_sem_take(frame.sem, K_NO_WAIT);
        if (err == 0) {
            err = can_send(frame.can_dev, &(frame.frame), K_MSEC(1), can_tx_callback, frame.sem);
            if (err) {
                LOG_ERR("Failed to send CAN frame: %d", err);
            }
            k_msgq_purge(msgq);
        } else {
            if (give_up) {
                k_msgq_purge(msgq);
                break;
            }
            k_sleep(K_USEC(300));
            give_up = true;
            continue;
        }
    }
    return err;
}

void dm_motor_control(const struct device *dev, enum motor_cmd cmd) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;

    struct can_frame frame;
    frame.id    = cfg->common.tx_id;
    frame.flags = 0;
    frame.dlc   = 8;

    int err    = 0;
    int can_id = get_can_id(dev);

    struct tx_frame tx_frame = {
        .can_dev = cfg->common.phy,
        .sem     = &tx_queue_sem[can_id],
    };
    switch (cmd) {
    case ENABLE_MOTOR:
        data->online = true;
        memcpy(frame.data, enable_frame, 8);
        tx_frame.frame = frame;

        err = dm_send_queued(&tx_frame, &dm_can_tx_msgq);
        break;
    case DISABLE_MOTOR:
        data->online = false;
        memcpy(frame.data, disable_frame, 8);
        tx_frame.frame = frame;

        err = dm_send_queued(&tx_frame, &dm_can_tx_msgq);
        break;
    case SET_ZERO_OFFSET:
        memcpy(frame.data, set_zero_frame, 8);
        tx_frame.frame = frame;

        err = dm_send_queued(&tx_frame, &dm_can_tx_msgq);
        break;
    case CLEAR_PID: memset(&data->params, 0, sizeof(data->params)); break;
    case CLEAR_ERROR:
        memcpy(frame.data, clear_error_frame, 8);
        tx_frame.frame = frame;

        err = dm_send_queued(&tx_frame, &dm_can_tx_msgq);
        break;
    }
    if (err != 0) {
        LOG_ERR("Failed to send CAN frame: %d", err);
    }
}

static void dm_motor_pack(const struct device *dev, struct can_frame *frame) {
    uint16_t                      pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint8_t                      *pbuf, *vbuf;
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;

    frame->id    = cfg->common.tx_id + data->tx_offset;
    frame->dlc   = 8;
    frame->flags = 0;
    switch (data->common.mode) {
    case MIT:
        pos_tmp = float_to_uint(data->target_angle, -cfg->p_max, cfg->p_max, 16);
        vel_tmp = float_to_uint(data->target_rpm, -cfg->v_max, cfg->v_max, 12);
        tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 12);
        kp_tmp  = float_to_uint(data->params.k_p, 0, 500, 12);
        kd_tmp  = float_to_uint(data->params.k_d, 0, 5, 12);

        frame->data[0] = (pos_tmp >> 8);
        frame->data[1] = pos_tmp;
        frame->data[2] = (vel_tmp >> 4);
        frame->data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        frame->data[4] = kp_tmp;
        frame->data[5] = (kd_tmp >> 4);
        frame->data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        frame->data[7] = tor_tmp;
        break;
    case PV:
        pbuf = (uint8_t *)&data->target_angle;
        vbuf = (uint8_t *)&data->target_rpm;

        frame->data[0] = *pbuf;
        frame->data[1] = *(pbuf + 1);
        frame->data[2] = *(pbuf + 2);
        frame->data[3] = *(pbuf + 3);
        frame->data[4] = *vbuf;
        frame->data[5] = *(vbuf + 1);
        frame->data[6] = *(vbuf + 2);
        frame->data[7] = *(vbuf + 3);
        break;
    case VO:
        vbuf = (uint8_t *)&data->target_rpm;

        frame->data[0] = *vbuf;
        frame->data[1] = *(vbuf + 1);
        frame->data[2] = *(vbuf + 2);
        frame->data[3] = *(vbuf + 3);
        break;
    default: break;
    }
}

float dm_motor_get_angle(const struct device *dev) {
    struct dm_motor_data *data = dev->data;
    return data->common.angle;
}

float dm_motor_get_speed(const struct device *dev) {
    struct dm_motor_data *data = dev->data;
    return data->common.rpm;
}

float dm_motor_get_torque(const struct device *dev) {
    struct dm_motor_data *data = dev->data;
    return data->common.torque;
}

int dm_motor_set_mode(const struct device *dev, enum motor_mode mode) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;
    char                          mode_str[10];

    data->common.mode = mode;

    switch (mode) {
    case MIT:
        strcpy(mode_str, "mit");
        data->tx_offset = 0x0;
        break;
    case PV:
        strcpy(mode_str, "pv");
        data->tx_offset = 0x100;
        break;
    case VO:
        strcpy(mode_str, "vo");
        data->tx_offset = 0x200;
        break;
    case MULTILOOP:
        data->online = false;
        return -ENOSYS;
        break;
    default: break;
    }

    for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.controller); i++) {
        if (cfg->common.controller[i] == NULL)
            break;
        if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
            const struct pid_single_config *params = pid_get_params(cfg->common.controller[i]);

            data->common.mode = mode;
            data->params.k_p  = params->k_p;
            data->params.k_d  = params->k_d;
            break;
        }
    }
    return 0;
}

int dm_motor_set_torque(const struct device *dev, float torque) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;

    data->target_rpm    = 0;
    data->target_angle  = 0;
    float torque_scaled = torque / cfg->gear_ratio;
    data->target_torque = torque_scaled;
    data->params.k_p    = 0;
    data->params.k_d    = 0;
    return 0;
}

int dm_motor_set_speed(const struct device *dev, float speed_rpm) {
    struct dm_motor_data *data = dev->data;

    data->target_rpm = speed_rpm;

    return 0;
}

// user must set speed before setting angle
int dm_motor_set_angle(const struct device *dev, float angle) {
    struct dm_motor_data *data = dev->data;

    data->target_angle = angle;

    return 0;
}

int get_motor_id(int id) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const struct device          *dev = motor_devices[i];
        const struct dm_motor_config *cfg = (const struct dm_motor_config *)(dev->config);
        if (cfg->common.rx_id == id) {
            return i;
        }
    }
    return -1;
}

static struct can_filter filters[CAN_COUNT];

static void dm_motor_ctrl_entry(void *arg1, void *arg2, void *arg3) {
    LOG_DBG("DM motor control thread started");
    struct can_frame tx_frame;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        int                           can_id = 0;
        const struct dm_motor_config *cfg =
            (const struct dm_motor_config *)(motor_devices[i]->config);

        for (int j = 0; j < CAN_COUNT; j++) {
            if (can_devices[j] == cfg->common.phy) {
                can_id = j;
                break;
            }
        }
        filters[can_id].id &= cfg->common.rx_id & 0xFF;
        filters[can_id].mask = ~(filters[can_id].mask ^ (cfg->common.rx_id & 0xFF)) & 0xFF;

        int err = can_start(cfg->common.phy);
        if (err) {
            LOG_ERR("Failed to start CAN device: %d", err);
        }
    }

    for (int i = 0; i < CAN_COUNT; i++) {
        filters[i].mask |= 0x700;
        const struct device *can_dev = can_devices[i];

        int err = can_add_rx_filter_msgq(can_dev, &dm_can_rx_msgq, &filters[i]);
        if (err < 0)
            LOG_ERR("Error adding CAN filter (err %d)", err);
        // If you recieved an error here, remember that 2# CAN of STM32 is in slave
        // mode and does not have an independent filter. If that is the issue, you
        // can ignore that.
    }

    k_sleep(K_MSEC(1000));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        dm_motor_control(motor_devices[i], ENABLE_MOTOR);
    }

    for (;;) {
        struct can_frame rx_frame;
        while (!k_msgq_get(&dm_can_rx_msgq, &rx_frame, K_NO_WAIT)) {
            int id = get_motor_id(rx_frame.id);
            if (id == -1) {
                LOG_ERR("Unknown motor ID: %d", rx_frame.id);
                break;
            }

            struct dm_motor_data *data = (struct dm_motor_data *)(motor_devices[id]->data);
            const struct dm_motor_config *cfg =
                (const struct dm_motor_config *)(motor_devices[id]->config);

            if (data->missed_times > 0) {
                data->missed_times--;
            }
            data->err = rx_frame.data[0] >> 4;

            float prev_angle   = data->common.angle;
            data->common.angle = (uint_to_float((rx_frame.data[1] << 8) | rx_frame.data[2],
                                                -cfg->p_max, cfg->p_max, 16)) *
                                 RAD2DEG;

            data->delta_deg_sum += data->common.angle - prev_angle;
            if (data->delta_deg_sum > 360) {
                data->common.round_cnt++;
                data->delta_deg_sum -= 360.0f;
            } else if (data->delta_deg_sum < -360) {
                data->common.round_cnt--;
                data->delta_deg_sum += 360.0f;
            }

            data->common.rpm = uint_to_float((rx_frame.data[3] << 4) | (rx_frame.data[4] >> 4),
                                             -cfg->v_max, cfg->v_max, 12);

            data->common.torque = uint_to_float(
                ((rx_frame.data[4] & 0xF) << 8) | rx_frame.data[5], -cfg->t_max, cfg->t_max, 12);
        }
        for (int i = 0; i < MOTOR_COUNT; i++) {
            struct dm_motor_data         *data = motor_devices[i]->data;
            const struct dm_motor_config *cfg  = motor_devices[i]->config;
            if (data->online) {
                int can_id = get_can_id(motor_devices[i]);
                int err    = k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT);
                if (err == 0) {
                    dm_motor_pack(motor_devices[i], &tx_frame);
                    struct tx_frame queued_frame = {
                        .can_dev = cfg->common.phy,
                        .sem     = &tx_queue_sem[can_id],
                        .frame   = tx_frame,
                    };
                    dm_send_queued(&queued_frame, &dm_can_tx_msgq);
                }
                if (++(data->missed_times) > 3) {
                    dm_motor_control(motor_devices[i], CLEAR_ERROR);
                } else if (data->missed_times > 5) {
                    LOG_ERR("Motor %d is not responding, setting it to offline...", i);
                    data->online = false;
                }
            } else if (data->missed_times <= 5) {
                dm_motor_control(motor_devices[i], CLEAR_ERROR);
                data->online = true;
                LOG_ERR("Motor %d is responding again, resuming...", i);
            }
        }
        dm_queue_proceed(&dm_can_tx_msgq);
        k_sleep(K_USEC(750));
    }
}

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)