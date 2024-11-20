/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_dji.h"
#include "syscalls/pid.h"
#include "zephyr/device.h"
#include <math.h>
#include <soc.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT dji_motor

LOG_MODULE_REGISTER(motor_dji, CONFIG_MOTOR_LOG_LEVEL);

const struct device *motor_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(MOTOR_PATH, DJI_DEVICE_POINTER, (, ))};

const struct device *can_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CAN_DEVICE_POINTER, (, ))};

#define CTRL_STRUCT_DATA(node_id)                                              \
  {                                                                            \
    .can_dev = DT_GET_CANPHY_BY_BUS(node_id), .target_rpm = {0}, .flags = 0,   \
    .info = {0}, .full[0] = false, .full[1] = false, .full[2] = false,         \
    .full[3] = false, .mask = 0,                                               \
  }

static int frames_id(int tx_id) {
  if (tx_id == 0x200) {
    return 0;
  } else if (tx_id == 0x1FF) {
    return 1;
  } else if (tx_id == 0x1FE) {
    return 2;
  } else if (tx_id == 0x2FE) {
    return 3;
  }
  return -1; // Return a default value if no match is found
}

static int txframe_id(int frames_id) {
    if(frames_id == 0) {
        return 0x200;
    } else if(frames_id == 1) {
        return 0x1FF;
    } else if(frames_id == 2) {
        return 0x1FE;
    } else if(frames_id == 3) {
        return 0x2FE;
    }
    return -1;
}

/**
 * @brief 将 float 类型转换为 int16_t，包含溢出处理和四舍五入。
 *
 * @param value 要转换的浮点数。
 * @return 转换后的 int16_t 值。
 */
static int16_t to16t(float value) {
  // 溢出处理
  if (value > INT16_MAX) {
    return INT16_MAX;
  } else if (value < INT16_MIN) {
    return INT16_MIN;
  } else {
    return (int16_t)value;
  }
}

static void can_send_entry(struct motor_controller *ctrl_struct, void *arg2,
                           void *arg3);
struct motor_controller motor_cans[CAN_COUNT] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CTRL_STRUCT_DATA, (, ))};

K_THREAD_DEFINE(dji_motor_ctrl_thread, CAN_SEND_STACK_SIZE, can_send_entry,
                motor_cans, can_devices, motor_devices, CAN_SEND_PRIORITY, 0,
                10);

static inline motor_id_t canbus_id(const struct device *dev) {
  struct dji_motor_data *data = dev->data;
  return data->canbus_id;
}

static inline motor_id_t motor_id(const struct device *dev) {
  const struct dji_motor_config *cfg = dev->config;
  return cfg->common.id - 1;
}

int8_t dji_set_speed(const struct device *dev, float speed_rpm) {
  struct dji_motor_data *data = dev->data;
  struct dji_motor_config *cfg = dev->config;
  uint8_t id = motor_id(dev);
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  ctrl_struct->target_rpm[id] = speed_rpm;
  for (int i = 0;
       i < sizeof(cfg->common.controller) / sizeof(cfg->common.controller[0]);
       i++) {
    if (cfg->common.controller[i] == NULL)
      break;
    if (pid_get_capability(cfg->common.controller[i], "speed")) {
      pid_calc(cfg->common.controller[i]);
      data->current_mode_index = i;
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
  return 0;
}

int8_t dji_set_angle(const struct device *dev, float angle) {
  struct dji_motor_data *data = dev->data;
  const struct dji_motor_config *cfg = dev->config;
  uint8_t id = motor_id(dev);
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  ctrl_struct->target_angle[id] = angle;
  for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.controller); i++) {
    if (cfg->common.controller[i] == NULL)
      break;
    if (pid_get_capability(cfg->common.controller[i], "angle")) {
      pid_calc(cfg->common.controller[i]);
      data->current_mode_index = i;
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
  return 0;
}

int8_t dji_set_torque(const struct device *dev, float torque) {
  struct dji_motor_data *data = dev->data;
  const struct dji_motor_config *cfg = dev->config;
  uint8_t id = motor_id(dev);
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  ctrl_struct->target_torque[id] = torque;
  for (int i = 0;
       i < sizeof(cfg->common.controller) / sizeof(cfg->common.controller[0]);
       i++) {
    if (cfg->common.controller[i] == NULL) {
        data->current_mode_index = -1;
        ctrl_struct->target_current[id] = 0;
        break;
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
  return 0;
}

float dji_set_zero(const struct device *dev) {
  float curr_angle = dji_get_angle(dev);
  struct dji_motor_data *data = dev->data;
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  uint8_t id = motor_id(dev);
  ctrl_struct->info[id].RAW_angle = 0;
  return curr_angle;
}

float dji_get_angle(const struct device *dev) {
  struct dji_motor_data *data = dev->data;
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  return ctrl_struct->info[motor_id(dev)].PCD_angle;
}

float dji_get_speed(const struct device *dev) {
  struct dji_motor_data *data = dev->data;
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  return ctrl_struct->info[motor_id(dev)].PCD_rpm;
}

float dji_get_torque(const struct device *dev) {
  struct dji_motor_data *data = dev->data;
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  return ctrl_struct->info[motor_id(dev)].PCD_torque;
}

int dji_init(const struct device *dev) {
  /*
                Each motor find their own controller and write their own data,
     which cannot be done at the first init because the other motors have not
     been instantiated.
  */
  if (dev) {
    const struct dji_motor_config *cfg = dev->config;
    struct dji_motor_data *data = (struct dji_motor_data *)dev->data;
    uint8_t frame_id = frames_id(cfg->common.tx_id);
    uint8_t id = motor_id(dev);
    data->ctrl_struct->mask[frame_id] |= 0xFF;
    data->ctrl_struct->mask[frame_id] ^= 1 << (id % 4);
    data->ctrl_struct->rx_ids[id] = cfg->common.rx_id;
    for (int i = 0;
         i < sizeof(cfg->common.controller) / sizeof(cfg->common.controller[0]);
         i++) {
      if (cfg->common.controller[i] == NULL)
        break;
      struct device *dev =
          (struct device*)cfg->common.controller[i];
      if (pid_get_capability(dev, "speed") == 0) {
        pid_reg_input(cfg->common.controller[i],
                      &data->ctrl_struct->info[id].PCD_rpm,
                      &data->ctrl_struct->target_rpm[id]);
        pid_reg_output(cfg->common.controller[i],
                       &data->ctrl_struct->target_torque[id]);
      } else if (pid_get_capability(dev, "angle") == 0) {
        pid_reg_input(cfg->common.controller[i],
                      &data->ctrl_struct->info[id].PCD_angle,
                      &data->ctrl_struct->target_angle[id]);
        pid_reg_output(cfg->common.controller[i],
                       &data->ctrl_struct->target_rpm[id]);
      } else {
        LOG_ERR("Unsupported motor mode");
        return -1;
      }
      pid_reg_time(cfg->common.controller[i],
                   &(data->ctrl_struct->info[id].curr_time),
                   &(data->ctrl_struct->info[id].prev_time));
    }
    data->current_mode_index = 0;
    data->ctrl_struct->motor_devs[id] = (struct device*) dev;
    data->ctrl_struct->info[id].prev_time = 0;
    data->ctrl_struct->flags |= 1 << id;
    data->ctrl_struct->mapping[frame_id][id % 4] = id;
    if (cfg->is_gm6020) {
      data->convert_num = GM6020_CONVERT_NUM;
    } else if (cfg->is_m3508) {
      data->convert_num = M3508_CONVERT_NUM;
    } else if (cfg->is_m2006) {
      data->convert_num = M2006_CONVERT_NUM;
    } else {
      LOG_ERR("Unsupported motor type");
    }

    if (!device_is_ready(cfg->common.phy))
      return -1;
    k_thread_start(dji_motor_ctrl_thread);
  }
  return 0;
}

void can_rx_callback(const struct device *can_dev, struct can_frame *frame,
                     void *user_data) {
  uint32_t current_time = k_cycle_get_32();
  struct motor_controller *ctrl_struct = (struct motor_controller *)user_data;
  struct can_frame rx_frame = *frame;
  // Suppose it is 3508/2006
  uint8_t id = (rx_frame.id & 0xF) - 1;
  // If RX_ID does not match, it should be GM6020
  if (ctrl_struct->rx_ids[id] != rx_frame.id && id > 4)
    id = (rx_frame.id & 0xF) - 5;
  // It should match, but in case we check again
  if (ctrl_struct->rx_ids[id] != rx_frame.id) {
    LOG_ERR("Unknown motor ID: %d, database: %d, received: %d", id, ctrl_struct->rx_ids[id], rx_frame.id);
    return;
  }
  int8_t bus_id = canbus_id(can_dev);
  uint32_t prev_time = ctrl_struct->info[id].curr_time;
  if (!ctrl_struct)
    return;
  struct motor_info *motor_info = &ctrl_struct[bus_id].info[id];
  if (!motor_info)
    return;
  // Store in RAW data. Process when API is called.
  // Using FPU in ISR is not recommended, since it requires action on registers
  motor_info->prev_angle = motor_info->angle;
  motor_info->angle = COMBINE_HL8(rx_frame.data[0], rx_frame.data[1]);
  motor_info->rpm = COMBINE_HL8(rx_frame.data[2], rx_frame.data[3]);
  motor_info->current = COMBINE_HL8(rx_frame.data[4], rx_frame.data[5]);
  motor_info->temperature = rx_frame.data[6];
  ctrl_struct[bus_id].flags |= 1 << id;
  ctrl_struct->info[id].curr_time = current_time;
  ctrl_struct->info[id].prev_time = prev_time;
  for (int i = 0; i < 3; i++) {
    if ((ctrl_struct[bus_id].mask[i] | ctrl_struct[bus_id].flags) ==
        0xFF) {
      //   ctrl_struct->flags ^= 0xF;
      ctrl_struct[bus_id].full[i] = 1;
      k_thread_resume(dji_motor_ctrl_thread);
      return;
    }
  }
}

static const struct can_filter filter20x = {
    .id = 0x200, .mask = 0x3F0, .flags = 0};

static void can_tx_callback(const struct device *can_dev, int error,
                            void *user_data) {
  struct k_sem *queue_sem = user_data;
  k_sem_give(queue_sem);
}

static int delta_degree(uint16_t angle, uint16_t prev_angle) {
  int delta = angle - prev_angle;
  if (angle < 2048 && prev_angle > 6144)
    delta += 8192;
  else if (angle > 6144 && prev_angle < 2048)
    delta -= 8192;
  return delta;
}

static void motor_calc(const struct device *dev) {
  const struct device *dev_temp = dev;
  struct dji_motor_data *data_temp = dev_temp->data;
  const struct dji_motor_config *config_temp = dev_temp->config;
  // Proceed the RAW data
  struct motor_info *info_temp =
      &data_temp->ctrl_struct[data_temp->canbus_id].info[motor_id(dev)];
  // Did we just pass a whole circle?
  info_temp->RAW_angle += delta_degree(info_temp->angle, info_temp->prev_angle);
  info_temp->RAW_angle %= (int)(8192 * config_temp->gear_ratio * 100);
  // I dont know why RAW_angle is 4 times of angle
  info_temp->PCD_angle =
      fmodf((float)(info_temp->RAW_angle) *
                convert[data_temp->convert_num][ANGLE2DEGREE] /
                (4 * config_temp->gear_ratio),
            360.0f);
  info_temp->PCD_rpm = info_temp->rpm *
                       convert[data_temp->convert_num][SPEED2RPM] /
                       config_temp->gear_ratio;
  info_temp->PCD_torque = info_temp->current *
                          convert[data_temp->convert_num][CURRENT2TORQUE] *
                          config_temp->gear_ratio;
  // If current_mode_index is -1, it means the motor is in torque control.
  // From the current mode index to the end of the controller array
  if(data_temp->current_mode_index != -1) {
    for (int i = data_temp->current_mode_index;
       i < SIZE_OF_ARRAY(config_temp->common.controller); i++) {
         if (config_temp->common.controller[i] == NULL)
            break;
        pid_calc(config_temp->common.controller[i]);
    }
  }
  data_temp->ctrl_struct[data_temp->canbus_id].target_current[motor_id(dev)] =
      to16t(data_temp->ctrl_struct[data_temp->canbus_id]
                .target_torque[motor_id(dev)] *
            convert[data_temp->convert_num][TORQUE2CURRENT] / config_temp->gear_ratio);
}

static struct k_sem tx_queue_sem;
struct can_frame txframe[CAN_COUNT][2];

static void can_send_entry(struct motor_controller *ctrl_struct, void *arg2,
                           void *arg3) {
  k_sem_init(&tx_queue_sem, 24, 24); // 初始化信号量
  struct device *can_dev = NULL;
  for (int i = 0; i < CAN_COUNT; i++) {
    can_dev = (struct device *)ctrl_struct[i].can_dev;
    can_start(can_dev);
    int err = can_add_rx_filter(can_dev, can_rx_callback, &ctrl_struct[i],
                                &filter20x);
    if (err != 0)
      LOG_ERR("Error adding CAN filter (err %d)", err);
    // If you recieved an error here, remember that 2# CAN of STM32 is in slave
    // mode and does not have an independent filter. If that is the issue, you
    // can ignore that.
  }
  int err = 0;

  k_sleep(K_MSEC(500));
  while (1) {
    for (int8_t i = 0; i < CAN_COUNT; i++) {
      for (int j = 0; j < 4; j++) {
        if (ctrl_struct[i].full[j] == 1) {
          uint8_t id_temp = ctrl_struct[i].mapping[0][0];
          for (int k = 0; k < 4; k++) {
            id_temp = ctrl_struct[i].mapping[j][k];
            // Check if recieved just now.
            if (id_temp < 8 && (ctrl_struct[i].flags & (1 << id_temp))) {
              motor_calc(ctrl_struct[i].motor_devs[id_temp]);
            }
          }
          txframe[i][j].id = txframe_id(j);
          txframe[i][j].dlc = 8;
          txframe[i][j].flags = 0;
          uint8_t data[8] = {0};
          for (int k = 0; k < 4; k++) {
            if ((ctrl_struct[i].mask[j] & (1U << k)) == 0) {
              data[k * 2] = HIGH_BYTE(
                  ctrl_struct[i].target_current[ctrl_struct[i].mapping[j][k]]);
              data[k * 2 + 1] = LOW_BYTE(
                  ctrl_struct[i].target_current[ctrl_struct[i].mapping[j][k]]);
            }
          }
          memcpy(txframe[i][j].data, data, sizeof(data));
          can_dev = (struct device *)ctrl_struct[i].can_dev;
          err = k_sem_take(&tx_queue_sem, K_NO_WAIT);
          if (err == 0)
            err = can_send(can_dev, &txframe[i][j], K_NO_WAIT, can_tx_callback,
                           &tx_queue_sem);
          if (err != 0 && err != -EAGAIN && err != -EBUSY)
            LOG_ERR("Error sending CAN frame (err %d)", err);
        }
      }
    }
    k_thread_suspend(dji_motor_ctrl_thread);
  }
}

DT_INST_FOREACH_STATUS_OKAY(DMOTOR_INST)



