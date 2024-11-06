/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_SINGLE_H
#define PID_SINGLE_H

#include "zephyr/devicetree.h"
#include "zephyr/drivers/counter.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pid.h>

#define DT_DRV_COMPAT pid_single

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)                               \
  {                                                                            \
    .k_p = DT_PROP(node_id, k_p) / 100.0,                                      \
    .k_i = DT_PROP(node_id, k_i) / 100.0,                                      \
    .k_d = DT_PROP(node_id, k_d) / 100.0,                                      \
  }

#include <limits.h>
#include <math.h>
#include <stdint.h>

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

static void single_pid_calc(const struct device *pid_dev) {
  struct pid_single_driver_config *pid_para = pid_dev->config;
  struct pid_single_driver_data *pid_data = pid_dev->data;
  if (pid_data->curr == NULL) {
    return;
  }
  float kp = pid_para->k_p;
  float ki = pid_para->k_i;
  float kd = pid_para->k_d;
  float err = (float)(*(pid_data->curr) - *(pid_data->ref));
  float deltaT =
      k_ticks_to_us_near64(*(pid_data->curr_time) - *(pid_data->prev_time));
  pid_data->err_integral += err * deltaT;
  pid_data->err_derivate = 1000000 * err / deltaT;
  *(pid_data->output) = to16t(
      kp * (err + ki * pid_data->err_integral + kd * pid_data->err_derivate));
  return;
}

static void single_pid_reg_input(const struct device *pid_dev, int16_t *curr,
                                 int16_t *ref) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->curr = curr;
  pid_data->ref = ref;
  return;
}

static void single_pid_reg_time(const struct device *pid_dev,
                                uint64_t *curr_time, uint64_t *prev_time) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->curr_time = curr_time;
  pid_data->prev_time = prev_time;
  return;
}

static void single_pid_reg_output(const struct device *pid_dev,
                                  int16_t *output) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->output = output;
  return;
}

static struct pid_driver_api pid_api_funcs = {
    .pid_calc = single_pid_calc,
    .pid_reg_input = single_pid_reg_input,
    .pid_reg_time = single_pid_reg_time,
    .pid_reg_output = single_pid_reg_output};

// static uintptr_t pid_addr(const struct device *dev, float *curr) {
//   struct pid_single_driver_data *data = dev->data;
//   data->curr = curr;
//   return (uintptr_t)data;
// }

#define PID_CONFIG_DEFINE(inst)                                                \
  static const struct pid_single_driver_config                                 \
      pid_single_driver_config_##inst =                                        \
          PID_SINGLE_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst));

#define PID_DATA_DEFINE(inst)                                                  \
  static struct pid_single_driver_data pid_single_driver_data_##inst;

#define PID_INST(inst)                                                         \
  PID_CONFIG_DEFINE(inst)                                                      \
  PID_DATA_DEFINE(inst)                                                        \
  PID_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL,                          \
                       &pid_single_driver_data_##inst,                         \
                       &pid_single_driver_config_##inst, POST_KERNEL,          \
                       CONFIG_MOTOR_INIT_PRIORITY, &pid_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

#endif // PID_SINGLE_H