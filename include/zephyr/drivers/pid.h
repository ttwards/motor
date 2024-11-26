/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_H
#define PID_H

#include "zephyr/toolchain.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>

// TODO: PID, ADRC, FSF, LQR, MPC, etc.

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio,  \
                             api, ...)                                         \
  DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api,       \
                   __VA_ARGS__)

#define NORMAL_PID 		0U
#define MIT_PID			1U

struct pid_single_driver_config {
  float k_p;
  float k_i;
  float k_d;
  char input[16];
};

struct pid_single_driver_data {
  float *ref;
  float *curr;
  float *mit_v_ref;
  float *mit_v_curr;
  float err_integral;
  float err_derivate;
  float ratio;
  int32_t *curr_time;
  int32_t *prev_time;
  float *output;
  int8_t mode;
};

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef void (*pid_api_calc_t)(const struct device *dev);

// typedef int32_t (*pid_api_addr_t)(const struct device *dev);

typedef void (*pid_api_reg_input_t)(const struct device *dev, float *curr,
                                    float *ref);

typedef void (*pid_api_reg_time_t)(const struct device *dev,
                                   uint32_t *curr_time, uint32_t *prev_time);

typedef void (*pid_api_reg_output_t)(const struct device *dev, float *output);

typedef bool (*pid_api_get_capability_t)(const struct device *dev, char *str);


/**
 * @brief Servo Motor driver API
 */
__subsystem struct pid_driver_api {
  pid_api_calc_t pid_calc;
  // pid_api_addr_t pid_addr;
  pid_api_reg_input_t pid_reg_input;
  pid_api_reg_time_t pid_reg_time;
  pid_api_reg_output_t pid_reg_output;
  pid_api_get_capability_t pid_get_capability;
};

__syscall void pid_calc(const struct device *dev);

static inline void z_impl_pid_calc(const struct device *dev) {
  const struct pid_driver_api *api = (const struct pid_driver_api *)dev->api;

  if (api->pid_calc != NULL) {
    api->pid_calc(dev);
  }
}
__syscall void pid_reg_input(const struct device *dev, float *curr, float *ref);

static inline void z_impl_pid_reg_input(const struct device *dev, float *curr,
                                        float *ref) {
  const struct pid_driver_api *api = (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_input != NULL) {
    api->pid_reg_input(dev, curr, ref);
  }
}

__syscall void pid_reg_time(const struct device *dev, uint32_t *curr_time,
                            uint32_t *prev_time);

static inline void z_impl_pid_reg_time(const struct device *dev,
                                       uint32_t *curr_time,
                                       uint32_t *prev_time) {
  const struct pid_driver_api *api = (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_time != NULL) {
    api->pid_reg_time(dev, curr_time, prev_time);
  }
}

__syscall void pid_reg_output(const struct device *dev, float *output);

static inline void z_impl_pid_reg_output(const struct device *dev,
                                         float *output) {
  const struct pid_driver_api *api = (const struct pid_driver_api *)dev->api;

  if (api->pid_reg_output != NULL) {
    api->pid_reg_output(dev, output);
  }
}

__syscall bool pid_get_capability(const struct device *dev, char *str);

static inline bool z_impl_pid_get_capability(const struct device *dev,
                                             char *str) {
  const struct pid_driver_api *api = (const struct pid_driver_api *)dev->api;

  if (api->pid_get_capability != NULL) {
    return api->pid_get_capability(dev, str);
  }
  return false;
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/pid.h>

#endif // PID_H