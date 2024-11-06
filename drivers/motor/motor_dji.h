/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "zephyr/logging/log.h"
#include "zephyr/sys/time_units.h"
#include <limits.h>
#include <soc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>

LOG_MODULE_REGISTER(motor_dji, CONFIG_MOTOR_LOG_LEVEL);

#define CAN_DEVICE_NAME DT_LABEL(DT_CHOSEN(zephyr_canbus))
#define CAN_MSG_DLC 8
#define SEND_INTERVAL K_MSECONDS(10)

#define FIXED_POINT_SHIFT 8
#define MULTIPLIER                                                             \
  (int)(0.4 * (1 << FIXED_POINT_SHIFT)) // 0.4 * 256 = 102.4 ≈ 102

#define CAN_SEND_STACK_SIZE 4096
#define CAN_SEND_PRIORITY -1

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

#define SIZE_OF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))

/** @brief Macros for each in dts */
#define MOTOR_NODE DT_NODELABEL(motor)
#define CAN_BUS_NODE DT_NODELABEL(canbus)

#define MOTOR_PATH DT_PATH(rm_motor)
#define CAN_BUS_PATH DT_PATH(canbus)

#define IS_DJI_COMPAT(node_id)                                                 \
  (DT_NODE_HAS_COMPAT(node_id, dji_m3508) ||                                   \
   DT_NODE_HAS_COMPAT(node_id, dji_m2006) ||                                   \
   DT_NODE_HAS_COMPAT(node_id, dji_m6020) ||                                   \
   DT_NODE_HAS_COMPAT(                                                         \
       node_id, dji_others)) // Add other DJI compatible strings as needed

#define DJI_DEVICE_POINTER(node_id) DEVICE_DT_GET(node_id)
#define CAN_DEVICE_POINTER(node_id) DEVICE_DT_GET(DT_PROP(node_id, can_device))

const struct device *motor_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(MOTOR_PATH, DJI_DEVICE_POINTER, (, ))};

const struct device *can_devices[] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CAN_DEVICE_POINTER, (, ))};

#define GET_CANPHY_POINTER_BY_IDX(node_id, idx)                                \
  DEVICE_DT_GET(DT_PHANDLE(DT_CHILD_BY_IDX(node_id, idx)))
#define CANPHY_BY_IDX(idx) GET_CANPHY_POINTER_BY_IDX(CAN_BUS_PATH, idx)

#define MOTOR_COUNT sizeof(motor_devices) / sizeof(motor_devices[0])
#define CAN_COUNT DT_NUM_INST_STATUS_OKAY(vnd_canbus)

#define GET_CAN_CHANNEL_IDT(node_id) DT_PHANDLE(node_id, can_channel)
#define GET_CAN_DEV(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_device))
#define GET_TX_ID(node_id) DT_PHANDLE(node_id, tx_addr)
#define GET_RX_ID(node_id) DT_PHANDLE(node_id, rx_addr)
#define GET_CTRL_ID(node_id) GET_TX_ID(node_id) & 0xF

#define CTRL_STRUCT_DATA(node_id)                                              \
  {                                                                            \
    .can_dev = DT_GET_CANPHY_BY_BUS(node_id), .target_rpm = {0}, .flags = 0,   \
    .info = {0}, .full[0] = false, .full[1] = false, .full[2] = false,         \
    .mask = 0,                                                                 \
  }

// extern k_tid_t dji_motor_ctrl_thread;

/*  canbus_id_t specifies the ID of the CAN bus the motor is on
        which is defined in motor_devices[] */
typedef uint8_t canbus_id_t;

/*  allmotor_id_t specifies the ID of the motor
        which is in the flags
        use flags |= 1 << all_motor_id*/
typedef uint16_t allmotor_id_t;

/*  motor_id_t specifies the ID of the motor in motor_controller struct
        find the rpm in motor_cans->target_rpm[canbus_id] */
typedef uint16_t motor_id_t;

struct motor_info {
  int16_t angle;
  int16_t current;
  int16_t torque;
  int16_t rpm;
  int16_t temperature;
  int64_t curr_time;
  int64_t prev_time;
};

struct motor_controller {
  const struct device *can_dev;
  int16_t target_angle[8];
  int16_t target_rpm[8];
  int16_t target_torque[8];
  int16_t target_current[8];
  /*
          There are 5 tx addresses
          0x1FF, 0x200 for M3508/M2006
          0x1FF, 0x2FF for GM6020 Voltage control
          0x1FE, 0x2FE for GM6020 Current control.
          !!!! Current control for GM6020 is deprecated !!!!
          The 3 nums in tx_enabled[] are: 0x200, 0x1FF, 0x2FF.
  */
  int rx_ids[8];
  bool full[3];
  uint8_t mapping[3][4];
  uint8_t flags;
  uint8_t mask[3];
  const struct device *motor_devs[8];
  struct motor_info info[8];
};

int frames_id(int tx_id) {
  switch (tx_id) {
  case 0x1FF:
    return 0;
  case 0x200:
    return 1;
  case 0x2FF:
    return 2;
  }
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

struct dji_motor_data {
  struct motor_driver_data common;
  canbus_id_t canbus_id;
  struct motor_controller *ctrl_struct;
  uint8_t current_mode_index;
};

struct dji_motor_config {
  struct motor_driver_config common;
};

static void can_send_entry(struct motor_controller *ctrl_struct, void *arg2,
                           void *arg3);
struct motor_controller motor_cans[CAN_COUNT] = {
    DT_FOREACH_CHILD_STATUS_OKAY_SEP(CAN_BUS_PATH, CTRL_STRUCT_DATA, (, ))};

K_THREAD_DEFINE(dji_motor_ctrl_thread, CAN_SEND_STACK_SIZE, can_send_entry,
                motor_cans, can_devices, motor_devices, CAN_SEND_PRIORITY, 0,
                10);

static inline motor_id_t canbus_id(const struct device *dev) {
  const struct motor_driver_config *cfg = dev->config;
  return (cfg->rx_id & 0xF) - 1;
}

static inline motor_id_t motor_id(const struct device *dev) {
  const struct motor_driver_config *cfg = dev->config;
  if (strcmp(cfg->compat, "dji_gm6020") == 1)
    return (cfg->rx_id & 0xF) - 5;
  else
    return (cfg->rx_id & 0xF) - 1;
}

static inline bool is_gm6020(const struct device *dev) {
  const struct motor_driver_config *cfg = dev->config;
  return strcmp(cfg->compat, "dji_gm6020") == 1;
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

static int16_t dji_set_speed(const struct device *dev, int16_t speed_rpm) {
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
    if (cfg->common.capabilities[i] == MOTOR_MODE_SPEED) {
      pid_calc(cfg->common.controller[i]);
      data->current_mode_index = i;
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
  return 0;
}

static int16_t dji_set_angle(const struct device *dev, int16_t angle) {
  struct dji_motor_data *data = dev->data;
  struct dji_motor_config *cfg = dev->config;
  uint8_t id = motor_id(dev);
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  ctrl_struct->target_angle[id] = to16t((angle % 360) * 8191 / 360);
  for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.controller); i++) {
    if (cfg->common.controller[i] == NULL)
      break;
    if (cfg->common.capabilities[i] == MOTOR_MODE_ANGLE) {
      pid_calc(cfg->common.controller[i]);
      data->current_mode_index = i;
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
  return 0;
}

static int16_t dji_set_torque(const struct device *dev, int16_t torque) {
  struct dji_motor_data *data = dev->data;
  struct dji_motor_config *cfg = dev->config;
  uint8_t id = motor_id(dev);
  struct motor_controller *ctrl_struct = &data->ctrl_struct[data->canbus_id];
  ctrl_struct->target_torque[id] = torque;
  for (int i = 0;
       i < sizeof(cfg->common.controller) / sizeof(cfg->common.controller[0]);
       i++) {
    if (cfg->common.controller[i] == NULL)
      break;
    if (cfg->common.capabilities[i] == MOTOR_MODE_TORQUE) {
      data->current_mode_index = i;
      pid_calc(cfg->common.controller[i]);
    }
  }
  // ctrl_struct->current[id] = pid_calc(dev);
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
      if (cfg->common.capabilities[i] == MOTOR_MODE_SPEED) {
        pid_reg_input(cfg->common.controller[i],
                      &data->ctrl_struct->info[id].rpm,
                      &data->ctrl_struct->target_rpm[id]);
        pid_reg_output(cfg->common.controller[i],
                       &data->ctrl_struct->target_torque[id]);
      } else if (cfg->common.capabilities[i] == MOTOR_MODE_TORQUE) {
        pid_reg_input(cfg->common.controller[i],
                      &data->ctrl_struct->info[id].torque,
                      &data->ctrl_struct->target_torque[id]);
        pid_reg_output(cfg->common.controller[i],
                       &data->ctrl_struct->target_current[id]);
      } else if (cfg->common.capabilities[i] == MOTOR_MODE_ANGLE) {
        pid_reg_input(cfg->common.controller[i],
                      &data->ctrl_struct->info[id].angle,
                      &data->ctrl_struct->target_rpm[id]);
        data->ctrl_struct->target_rpm[id] = 666;
        pid_reg_output(cfg->common.controller[i],
                       &data->ctrl_struct->target_rpm[id]);
      } else {
        LOG_ERR("Unsupported motor mode");
        return -1;
      }
      pid_reg_time(cfg->common.controller[i],
                   &data->ctrl_struct->info[id].curr_time,
                   &data->ctrl_struct->info[id].prev_time);
    }
    data->current_mode_index = 0;
    data->ctrl_struct->motor_devs[id] = dev;
    data->ctrl_struct->info[id].prev_time = 0;
    data->ctrl_struct->flags |= 1 << id;
    data->ctrl_struct->mapping[frame_id][id % 4] = id;
    if (!device_is_ready(cfg->common.phy))
      return -1;
    k_thread_start(dji_motor_ctrl_thread);
  }
  return -2;
}

void can_rx_callback(const struct device *can_dev, struct can_frame *frame,
                     void *user_data) {
  uint64_t current_time = k_cycle_get_64();
  struct motor_controller *ctrl_struct = (struct motor_controller *)user_data;
  struct can_frame rx_frame = *frame;
  uint8_t id = (rx_frame.id & 0xF) - 1;
  if (ctrl_struct->rx_ids[id] != rx_frame.id)
    id = (rx_frame.id & 0xF) - 5;
  uint8_t canbus_id = 0;
  for (int i = 0; i < CAN_COUNT; i++) {
    if (can_dev == ctrl_struct[i].can_dev) {
      canbus_id = i;
      break;
    }
  }
  uint64_t prev_time = ctrl_struct->info[id].curr_time;
  if (!ctrl_struct)
    return;
  struct motor_info *motor_info = &ctrl_struct[canbus_id].info[id];
  if (!motor_info)
    return;
  motor_info->angle = COMBINE_HL8(rx_frame.data[0], rx_frame.data[1]);
  motor_info->rpm = COMBINE_HL8(rx_frame.data[2], rx_frame.data[3]);
  motor_info->torque =
      ((COMBINE_HL8(rx_frame.data[4], rx_frame.data[5])) * MULTIPLIER) >>
      FIXED_POINT_SHIFT;
  motor_info->temperature = rx_frame.data[6];
  ctrl_struct[canbus_id].flags |= 1 << id;
  ctrl_struct->info[id].curr_time = current_time;
  ctrl_struct->info[id].prev_time = prev_time;
  for (int i = 0; i < 3; i++) {
    if ((ctrl_struct[canbus_id].mask[i] | ctrl_struct[canbus_id].flags) ==
        0xFF) {
      //   ctrl_struct->flags ^= 0xF;
      ctrl_struct[canbus_id].full[i] = 1;
      k_thread_resume(dji_motor_ctrl_thread);
      return;
    }
  }
}

static const struct can_filter filter20x = {
    .id = 0x200, .mask = 0x3F0, .flags = 0};

static inline void can_calc(struct can_frame *frame, uint16_t tx_id,
                            int16_t *currents) {}

static void can_tx_callback(const struct device *can_dev, int error,
                            void *user_data) {
  struct k_sem *queue_sem = user_data;
  k_sem_give(queue_sem);
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
    for (int j = 0; j < 2; j++) {
      // can_calc(txframe[i], 0x200, ctrl_struct[i].current);
    }
  }
  int err = 0;

  k_sleep(K_MSEC(500));
  while (1) {
    for (int8_t i = 0; i < CAN_COUNT; i++) {
      for (int j = 0; j < 3; j++) {
        if (ctrl_struct[i].full[j] == 1) {
          uint8_t id_temp = ctrl_struct[i].mapping[0][0];
          for (int k = 0; k < 4; k++) {
            id_temp = ctrl_struct[i].mapping[j][k];
            if (id_temp < 8 && ctrl_struct[i].flags & (1 << id_temp)) {
              struct device *dev_temp = ctrl_struct[i].motor_devs[id_temp];
              struct dji_motor_data *data_temp = dev_temp->data;
              struct dji_motor_config *config_temp = dev_temp->config;
              for (int l = data_temp->current_mode_index;
                   l < SIZE_OF_ARRAY(config_temp->common.capabilities); l++) {
                if (config_temp->common.controller[l] == NULL)
                  break;
                pid_calc(config_temp->common.controller[l]);
              }
            }
          }
          txframe[i][j].id = ((0x1FF + (j & 0x1)) + ((j & 0x2) << 7));
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
          if (err != 0)
            LOG_ERR("Error sending CAN frame (err %d)", err);
        }
      }
    }
  }
  k_thread_suspend(dji_motor_ctrl_thread);
}

#endif // MOTOR_DJI_H