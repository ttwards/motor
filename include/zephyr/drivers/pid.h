/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_H
#define PID_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

// TODO: PID, ADRC, FSF, LQR, MPC, etc.

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)            \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define NORMAL_PID 0U
#define MIT_PID    1U

#define STATIC_VOID __attribute__((unused)) static void
#define STATIC __attribute__((unused)) static

#define PID_INS_CAT_NAME(node_name, name) DT_CAT(node_name, name)
#define PID_INS_NAME(node_id, name) PID_INS_CAT_NAME(DT_NODE_FULL_NAME_UNQUOTED(node_id), name)

#define PID_NEW_INSTANCE(node_id, name)                                                            \
	struct pid_data PID_INS_NAME(node_id, name) = {                                            \
		.pid_dev = DEVICE_DT_GET(node_id),                                                 \
	};

struct pid_config {
	float k_p;
	float k_i;
	float k_d;
	bool mit;
};

struct pid_data {
	float *ref;
	float *curr;

	float *detri_ref;
	float *detri_curr;

	float err_integral;
	float err_derivate;
	float ratio;
	struct device *pid_dev;
	int32_t *curr_time;
	int32_t *prev_time;
	float *output;
};

static bool float_equal(float a, float b)
{
	return fabsf(a - b) < 0.0001f;
}

STATIC_VOID pid_calc(struct pid_data *data)
{
	const struct device *dev = data->pid_dev;
	if (dev == NULL) {
		return;
	}
	const struct pid_config *pid_para = dev->config;
	if (!pid_para->mit) {
		if (data->curr == NULL) {
			return;
		}
		float kp = pid_para->k_p;
		float ki = pid_para->k_i;
		float kd = pid_para->k_d;
		float err = *(data->ref) - *(data->curr);
		float deltaT = k_cyc_to_us_near32(*(data->curr_time) - *(data->prev_time));
		if (!float_equal(ki, 0)) {
			data->err_integral += (err * deltaT) / (1000000 * ki);
		}
		if (!float_equal(kd, 0)) {
			data->err_derivate = kd * err / deltaT;
		}
		//   LOG_INF("integral: %d, derivate: %d", to16t(ki * (err * deltaT) / 1000000),
		//           to16t(kd * 1000000 * err / deltaT));
		*(data->output) = kp * (err + data->err_integral + data->err_derivate);

	} else {
		if (data->curr == NULL) {
			return;
		}
		float kp = pid_para->k_p;
		float ki = pid_para->k_i;
		float kd = pid_para->k_d;
		float err = *(data->ref) - *(data->curr);
		float deltaT = k_cyc_to_us_near32(*(data->curr_time) - *(data->prev_time));
		if (!float_equal(ki, 0)) {
			data->err_integral += (err * deltaT) / (1000000 * ki);
		}
		if (!float_equal(kd, 0)) {
			data->err_derivate =
				kd * (*(data->detri_ref) - *(data->detri_curr)) / deltaT;
		}

		*(data->output) = kp * (err + data->err_integral + data->err_derivate);
		return;
	}
}

STATIC_VOID pid_reg_input(struct pid_data *data, float *curr, float *ref)
{
	if (data == NULL) {
		return;
	}
	data->curr = curr;
	data->ref = ref;
}

STATIC_VOID pid_reg_output(struct pid_data *data, float *output)
{
	if (data == NULL) {
		return;
	}
	data->output = output;
}

STATIC_VOID pid_reg_time(struct pid_data *data, uint32_t *curr_cyc, uint32_t *prev_cyc)
{
	if (data == NULL) {
		return;
	}
	data->curr_time = curr_cyc;
	data->prev_time = prev_cyc;
}

STATIC_VOID mit_reg_detri_input(struct pid_data *data, float *detri_curr, float *detri_ref)
{
	if (data == NULL) {
		return;
	}
	data->detri_curr = detri_curr;
	data->detri_ref = detri_ref;
}

STATIC const struct pid_config *pid_get_params(struct pid_data *data)
{
	const struct device *dev = data->pid_dev;
	if (dev == NULL) {
		return NULL;
	}
	return dev->config;
}

#ifdef __cplusplus
}
#endif

#endif // PID_H