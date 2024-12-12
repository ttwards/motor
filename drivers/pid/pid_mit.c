/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_MIT_H
#define PID_MIT_H

#include "zephyr/devicetree.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT pid_mit

LOG_MODULE_REGISTER(pid_mit, CONFIG_MOTOR_LOG_LEVEL);

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)                                                 \
    {                                                                                            \
        .k_p = DT_PROP(node_id, k_p) / 10000.0f,                                                 \
        .k_i = DT_PROP(node_id, k_i) / 100.0f,                                                   \
        .k_d = DT_PROP(node_id, k_d) / 100.0f,                                                   \
    }

static bool float_equal(float a, float b) { return fabsf(a - b) < 0.0001f; }

struct pid_mit_data {
    float   *ref;
    float   *detri_ref;
    float   *curr;
    float   *detri_curr;
    float    err_integral;
    float    err_derivate;
    float    ratio;
    int32_t *curr_time;
    int32_t *prev_time;
    float   *output;
};

struct pid_mit_config {
    const struct pid_single_config common;
};

static const struct pid_single_config *pid_mit_get_params(const struct device *pid_dev) {
    return &((struct pid_mit_config *)pid_dev->config)->common;
}

static void pid_mit_calc(const struct device *pid_dev) {
    const struct pid_mit_config *pid_para = pid_dev->config;
    struct pid_mit_data         *pid_data = pid_dev->data;
    if (pid_data->curr == NULL) {
        return;
    }
    float kp     = pid_para->common.k_p;
    float ki     = pid_para->common.k_i;
    float kd     = pid_para->common.k_d;
    float err    = *(pid_data->ref) - *(pid_data->curr);
    float deltaT = k_cyc_to_us_near32(*(pid_data->curr_time) - *(pid_data->prev_time));
    if (!float_equal(ki, 0))
        pid_data->err_integral += (err * deltaT) / (1000000 * ki);
    if (!float_equal(kd, 0)) {
        pid_data->err_derivate = kd * (*(pid_data->detri_ref) - *(pid_data->detri_curr)) / deltaT;
    }
    //   LOG_INF("integral: %d, derivate: %d", to16t(ki * (err * deltaT) / 1000000),
    //           to16t(kd * 1000000 * err / deltaT));
    *(pid_data->output) = kp * (err + pid_data->err_integral + pid_data->err_derivate);
    return;
}

static void pid_mit_reg_input(const struct device *pid_dev, float *curr, float *ref) {
    struct pid_mit_data *pid_data = pid_dev->data;
    pid_data->curr                = curr;
    pid_data->ref                 = ref;
    return;
}

static void pid_mit_reg_detri(const struct device *pid_dev, float *curr, float *ref) {
    struct pid_mit_data *pid_data = pid_dev->data;
    pid_data->detri_curr          = curr;
    pid_data->detri_ref           = ref;
    return;
}

static void pid_mit_reg_time(const struct device *pid_dev, uint32_t *curr_time,
                             uint32_t *prev_time) {
    struct pid_mit_data *pid_data = pid_dev->data;
    pid_data->curr_time           = curr_time;
    pid_data->prev_time           = prev_time;
    return;
}

static void pid_mit_reg_output(const struct device *pid_dev, float *output) {
    struct pid_mit_data *pid_data = pid_dev->data;
    pid_data->output              = output;
    return;
}

static struct pid_driver_api pid_api_funcs = {.pid_calc       = pid_mit_calc,
                                              .pid_reg_input  = pid_mit_reg_input,
                                              .pid_reg_detri  = pid_mit_reg_detri,
                                              .pid_reg_time   = pid_mit_reg_time,
                                              .pid_reg_output = pid_mit_reg_output,
                                              .pid_get_params = pid_mit_get_params};

#define PID_CONFIG_DEFINE(inst)                                                                  \
    static const struct pid_mit_config pid_mit_cfg_##inst = {                                    \
        .common = PID_SINGLE_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))};

#define PID_DATA_DEFINE(inst)                                                                    \
    static struct pid_mit_data pid_mit_data_##inst = {.err_integral = 0,                         \
                                                      .err_derivate = 0,                         \
                                                      .curr         = NULL,                      \
                                                      .ref          = NULL,                      \
                                                      .output       = NULL,                      \
                                                      .curr_time    = NULL,                      \
                                                      .prev_time    = NULL,                      \
                                                      .detri_curr   = NULL,                      \
                                                      .detri_ref    = NULL};

#define PID_INST(inst)                                                                           \
    PID_CONFIG_DEFINE(inst)                                                                      \
    PID_DATA_DEFINE(inst)                                                                        \
    PID_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, &pid_mit_data_##inst,                    \
                         &pid_mit_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,           \
                         &pid_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

#endif // PID_MIT_H