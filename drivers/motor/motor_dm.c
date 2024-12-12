#include <cstdint>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "motor_dm.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/pid.h"

#define DT_DRV_COMPAT dm_motor

LOG_MODULE_REGISTER(motor_dm, CONFIG_MOTOR_LOG_LEVEL);

#define DM_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(DM_MOTOR_POINTER)};

#define CAN_BUS_PATH                DT_PATH(canbus)
#define CAN_DEVICE_POINTER(node_id) DEVICE_DT_GET(DT_PROP(node_id, can_device))
const struct device *can_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CAN_DEVICE_POINTER, (, ))};

uint8_t array_enable_motor[8]     = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
uint8_t array_disable_motor[8]    = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
uint8_t array_save_zero_offset[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
uint8_t array_error_clear[8]      = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

/**
 * @brief 将 float 类型转换为 int16_t，包含溢出处理和四舍五入。
 *
 * @param value 要转换的浮点数。
 * @return 转换后的 int16_t 值。
 */
static int16_t to16t(float value, float min, float max) {
    min = (min < INT16_MIN) ? INT16_MIN : min;
    max = (max > INT16_MAX) ? INT16_MAX : max;
    // 溢出处理
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    } else {
        return (int16_t)value;
    }
}

static int16_t to12t(float value, float min, float max) {
    min = (min < INT12_MIN) ? INT12_MIN : min;
    max = (max > INT12_MAX) ? INT12_MAX : max;
    // 溢出处理
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    } else {
        return (int16_t)value;
    }
}

static inline uint16_t scale(float value, float v_max, float v_min, int digits) {
    int   range     = (1 << digits) - 1;
    float scale_sum = range / (v_max - v_min);
    int   scaled    = scale_sum * value;
    if (scaled > range)
        scaled = range;
    return scaled;
}

void dm_motor_pack(struct device *dev, struct can_frame *frame) {
    uint16_t              pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint8_t              *pbuf, *vbuf;
    struct dm_motor_data *data = dev->data;
    switch (data->common.mode) {
    case MIT:
        pos_tmp = scale(data->target_angle, -cfg->p_max, cfg->p_max, 16);
        vel_tmp = scale(data->target_rpm, -cfg->v_max, cfg->v_max, 12);
        tor_tmp = scale(data->target_torque, -cfg->t_max, cfg->t_max, 12);
        kp_tmp  = scale(data->params.k_p, DM_J4310_2EC_KP_MIN, DM_J4310_2EC_KP_MAX, 12);
        kd_tmp  = scale(data->params.k_d, DM_J4310_2EC_KD_MIN, DM_J4310_2EC_KD_MAX, 12);

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

int8_t dm_motor_set_mode(const struct device *dev, enum motor_mode mode) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;
    char                          mode_str[10];

    data->mode = mode;

    switch (mode) {
    case MIT: strcpy(mode_str, "mit"); break;
    case PV: strcpy(mode_str, "pv"); break;
    case VO: strcpy(mode_str, "vo"); break;
    case MULTILOOP: strcpy(mode_str, "multiloop"); break;
    default: break;
    }

    for (int i = 0; i < sizeof(cfg->common.controller) / sizeof(cfg->common.controller[0]); i++) {
        if (cfg->common.controller[i] == NULL)
            break;
        if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
            pid_calc(cfg->common.controller[i]);
            data->current_mode_index = i;
            data->mode               = mode;
            data->params.k_p         = 0;
            data->params.k_d         = 0;
            break;
        }
    }
    return 0;
}

int8_t dm_motor_set_torque(const struct device *dev, float torque) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;

    data->target_rpm    = 0;
    data->target_angle  = 0;
    float torque_scaled = torque / cfg->gear_ratio;
    data->target_torque = torque_scaled;
    data->params.k_p    = 0;
    data->params.k_d    = 0;
    dm_motor_set_mode(dev, MIT);
    return 0;
}

int8_t dm_motor_set_speed(const struct device *dev, float speed_rpm) {
    struct dm_motor_data         *data = dev->data;
    const struct dm_motor_config *cfg  = dev->config;

    data->target_rpm = speed_rpm;
    dm_motor_set_mode(dev, VO);

    return 0;
}

// user must set speed before setting angle
int8_t dm_motor_set_angle(const struct device *dev, float angle) {
    struct dm_motor_data *data = dev->data;

    data->target_angle = angle;

    return 0;
}

float DM_motor::motor_get_current_rounds() {
    if (reverse) {
        return (-data.current_round);
    } else {
        return (data.current_round);
    }
}

void dm_motor_error_process() {
    uint8_t dis_cnt = 0;
    uint8_t en_cnt  = 0;
    if (data.err >= 8) {
        while (dis_cnt < 10) {
            this->motor_control(clear_error);
            dis_cnt++;
            osDelay(1);
        }

        while (en_cnt < 10) {
            this->motor_control(enable_motor);
            en_cnt++;
            osDelay(1);
        }
    }
}

int get_motor_id(int id, struct motor_controller *ctrl_struct) {
    for (int i = 0; i < CAN_COUNT; i++) {
        for (int j = 0; j < 8; j++) {
            if ((ctrl_struct[i].ids[j]) == id) {
                return j;
            }
        }
    }
    return -1;
}

void dm_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data) {
    struct motor_controller *ctrl_struct = (struct motor_controller *)user_data;

    int id = get_motor_id(frame->id, ctrl_struct);
    if (id == -1) {
        LOG_ERR("Unknown motor ID: %d", frame->id);
        return;
    }

    struct dm_motor_data *data = (struct dm_motor_data *)(ctrl_struct->motor_devs[id]->data);
    const struct dm_motor_config *cfg =
        (const struct dm_motor_config *)(ctrl_struct->motor_devs[id]->config);

    data->err = frame->data[0] >> 4;

    data->RAWangle      = (frame->data[1] << 8) | frame->data[2];
    data->RAWrpm        = (frame->data[3] << 4) | (frame->data[4] >> 4);
    data->RAWtorque     = ((frame->data[4] & 0xF) << 8) | frame->data[5];
    data->RAWtemp_mos   = frame->data[6];
    data->RAWtemp_rotor = frame->data[7];
}

bool DM_motor::motor_reset() {
    data.offset_ecd   = data.ecd;
    data.total_ecd    = 0;
    data.total_rounds = 0;
    data.round_cnt    = 0;
    return (true);
}

void dm_motor_calc(struct device *dev) {

    /* initial value */
    if (dm_motor->init_offset == false) {
        if (dm_motor->type == DM_J4310_2EC) {
            ptr->current_speed = //-1-1
                uint_to_float(data->vel, -cfg->v_max, cfg->v_max, 12) /
                cfg->v_max; // (-30.0,30.0)
        }

        ptr->current_round = // 0 - 2*P_MAX
            (uint_to_float(data->pos, -cfg->p_max, cfg->p_max, 16) +
             cfg->p_max) *
            RAD2ROUND; // (-3.14,3.14)

        ptr->offset_round = ptr->current_round;
        return;
    }
    //误差改小,溢出cnt
    ptr->last_round    = ptr->current_round; //电机编码反馈值
    ptr->current_round =                     // (0 - 2*P_MAX)/(2*PI)
        (uint_to_float(data->pos, -cfg->p_max, cfg->p_max, 16) +
         cfg->p_max) *
        RAD2ROUND; // (-3.14,3.14)
    if (ptr->current_round - ptr->last_round > cfg->p_max * RAD2ROUND) {
        ptr->round_cnt = ptr->round_cnt - 1;
        ptr->delta_rounds =
            ptr->current_round - ptr->last_round - (2.0f * cfg->p_max * RAD2ROUND);
    } else if (ptr->current_round - ptr->last_round < -cfg->p_max * RAD2ROUND) {
        ptr->round_cnt = ptr->round_cnt + 1;
        ptr->delta_rounds =
            ptr->current_round - ptr->last_round + (2.0f * cfg->p_max * RAD2ROUND);
    } else {
        ptr->delta_rounds = ptr->current_round - ptr->last_round;
    }
    ptr->total_rounds = (float)ptr->round_cnt * (2.0f * cfg->p_max * RAD2ROUND) +
                        ptr->current_round - ptr->offset_round;

    ptr->last_speed = ptr->current_speed;
    if (dm_motor->type == DM_J4310_2EC) {
        ptr->current_speed = //-1-1
            uint_to_float(data->vel, -cfg->v_max, cfg->v_max, 12) /
            cfg->v_max; // (-30.0,30.0)
    }

    ptr->torque = uint_to_float(data->torque, -cfg->t_max, cfg->t_max,
                                12); // (-10.0,10.0)
}

void DM_motor::motor_control(uint32_t cmd) {
    switch (cmd) {
    case enable_motor: memcpy(frame->data, array_enable_motor, 8); break;

    case disable_motor: memcpy(frame->data, array_disable_motor, 8); break;

    case save_zero_offset: memcpy(frame->data, array_save_zero_offset, 8); break;

    case clear_pid:
        velPid.pid_clear();
        posPid.pid_clear();
        break;

    case disable_offset:
        data.offset_ecd   = 0;
        data.offset_round = 0;
        break;

    case clear_error: memcpy(frame->data, array_error_clear, 8); break;
    }
}

void DM_motor_service(void *argument) {
    can_device_transmit DM_motor_can1_service(&hcan1);
    can_device_transmit DM_motor_can2_service(&hcan2);
    for (;;) {
        for (int i = 1; i <= 32; i++) {

            if (DM_motor_can1_enable_list[i] == 1) {
                DM_motor_can1_service.set_id(DM_motor_can1_tx_id[i]);
                DM_motor_can1_service.set_buf_address(DM_motor_can1_total_data[i]);
                DM_motor_can1_service.can_send_msg();
            }

            if (DM_motor_can2_enable_list[i] == 1) {
                DM_motor_can2_service.set_id(DM_motor_can2_tx_id[i]);
                DM_motor_can2_service.set_buf_address(DM_motor_can2_total_data[i]);
                DM_motor_can2_service.can_send_msg();
            }
        }
        osDelay(1);
    }
}

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)