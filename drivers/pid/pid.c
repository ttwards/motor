/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_SINGLE_H
#define PID_SINGLE_H

#include "zephyr/devicetree.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pid.h>

#define DT_DRV_COMPAT pid_single

#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(pid, CONFIG_MOTOR_LOG_LEVEL);

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)    \
  {                                                 \
    .k_p = DT_PROP(node_id, k_p) / 10000.0f,         \
    .k_i = DT_PROP(node_id, k_i) / 100.0f,          \
    .k_d = DT_PROP(node_id, k_d) / 100.0f,          \
    .input = DT_PROP(node_id, input),               \
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

bool float_equal(float a, float b) { return fabsf(a - b) < 0.0001f; }

static void single_pid_calc(const struct device *pid_dev) {
  const struct pid_single_driver_config *pid_para = pid_dev->config;
  struct pid_single_driver_data *pid_data = pid_dev->data;
  if (pid_data->curr == NULL) {
    return;
  }
  float kp = pid_para->k_p;
  float ki = pid_para->k_i;
  float kd = pid_para->k_d;
  float err = *(pid_data->ref) - *(pid_data->curr);
  float deltaT =
      k_cyc_to_us_near32(*(pid_data->curr_time) - *(pid_data->prev_time));
  float delta_i = (err * deltaT) / (1000000 * ki);
  float delta_d = kd * err / deltaT;
  if (!float_equal(ki, 0))
    pid_data->err_integral += delta_i;
  if (!float_equal(kd, 0))
    pid_data->err_derivate = delta_d;
//   LOG_INF("integral: %d, derivate: %d", to16t(ki * (err * deltaT) / 1000000),
//           to16t(kd * 1000000 * err / deltaT));
  *(pid_data->output) =
      kp * (err + pid_data->err_integral + pid_data->err_derivate);
  return;
}

static bool single_pid_get_capability(const struct device *dev, char *str) {
    const struct pid_single_driver_config *cfg = dev->config;
    return (strcmp(cfg->input, str) == 0) ? true : false;
}

static void single_pid_reg_input(const struct device *pid_dev, float *curr,
                                 float *ref) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->curr = curr;
  pid_data->ref = ref;
  return;
}

static void single_pid_reg_time(const struct device *pid_dev,
                                uint32_t *curr_time, uint32_t *prev_time) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->curr_time = curr_time;
  pid_data->prev_time = prev_time;
  return;
}

static void single_pid_reg_output(const struct device *pid_dev,
                                  float *output) {
  struct pid_single_driver_data *pid_data = pid_dev->data;
  pid_data->output = output;
  return;
}

static struct pid_driver_api pid_api_funcs = {
    .pid_calc = single_pid_calc,
    .pid_reg_input = single_pid_reg_input,
    .pid_reg_time = single_pid_reg_time,
    .pid_reg_output = single_pid_reg_output,
    .pid_get_capability = single_pid_get_capability,    
};

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