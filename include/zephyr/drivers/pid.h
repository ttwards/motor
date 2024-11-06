/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <errno.h>

// TODO: PID, ADRC, FSF, LQR, MPC, etc.

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio,  \
                             api, ...)                                         \
  DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api,       \
                   __VA_ARGS__)

struct pid_single_driver_config {
  double k_p;
  double k_i;
  double k_d;
};

struct pid_single_driver_data {
  int16_t *ref;
  int16_t *curr;
  float err_integral;
  float err_derivate;
  int64_t *curr_time;
  int64_t *prev_time;
  int16_t *output;
};

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef void (*pid_api_calc_t)(const struct device *dev);

// typedef int32_t (*pid_api_addr_t)(const struct device *dev);

typedef void (*pid_api_reg_input_t)(const struct device *dev, int16_t *curr, int16_t *ref);

typedef void (*pid_api_reg_time_t)(const struct device *dev, uint64_t *curr_time, uint64_t *prev_time);

typedef void (*pid_api_reg_output_t)(const struct device *dev, int16_t *output);

/**
 * @brief Servo Motor driver API
 */
__subsystem struct pid_driver_api {
  pid_api_calc_t pid_calc;
  // pid_api_addr_t pid_addr;
  pid_api_reg_input_t pid_reg_input;
  pid_api_reg_time_t pid_reg_time;
  pid_api_reg_output_t pid_reg_output;
};

__syscall void pid_calc(const struct device *dev);

static inline void z_impl_pid_calc(const struct device *dev)
{
  const struct pid_driver_api *api =
    (const struct pid_driver_api *)dev->api;

  if (api->pid_calc != NULL) {
    api->pid_calc(dev);
  }
}
__syscall void pid_reg_input(const struct device *dev, int16_t *curr, int16_t *ref);

static inline void z_impl_pid_reg_input(const struct device *dev, int16_t *curr, int16_t *ref)
{
  const struct pid_driver_api *api =
    (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_input != NULL) {
    api->pid_reg_input(dev, curr, ref);
  }
}

__syscall void pid_reg_time(const struct device *dev, uint64_t *curr_time, uint64_t *prev_time);

static inline void z_impl_pid_reg_time(const struct device *dev, uint64_t *curr_time, uint64_t *prev_time)
{
  const struct pid_driver_api *api =
    (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_time != NULL) {
    api->pid_reg_time(dev, curr_time, prev_time);
  }
}

__syscall void pid_reg_output(const struct device *dev, int16_t *output);

static inline void z_impl_pid_reg_output(const struct device *dev, int16_t *output)
{
  const struct pid_driver_api *api =
    (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_output != NULL) {
    api->pid_reg_output(dev, output);
  }
}



#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/pid.h>

#endif // PID_H