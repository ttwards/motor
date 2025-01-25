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

#define PID_NEW_INSTANCE(node_id, name, ref, curr, curr_time, prev_time, output)                   \
	struct pid_single_data DT_NODE_FULL_NAME_UNQUOTED(node_id)##name = {.ref = ref,.curr=curr, .pid_dev =DEVICE_DT_GET\
	\	
	}

struct pid_config {
	float k_p;
	float k_i;
	float k_d;
	bool mit;
};

struct pid_single_data {
	float *ref;
	float *curr;
	float err_integral;
	float err_derivate;
	float ratio;
	struct device *pid_dev;
	int32_t *curr_time;
	int32_t *prev_time;
	float *output;
};

struct pid_mit_data {
	float *ref;
	float *detri_ref;
	float *curr;
	float *detri_curr;
	float err_integral;
	float err_derivate;
	float ratio;
	int32_t *curr_time;
	int32_t *prev_time;
	float *output;
};

static bool float_equal(float a, float b)
{
	return fabsf(a - b) < 0.0001f;
}

static void pid_calc(const struct device *dev, void *data)
{
	if (dev == NULL) {
		return;
	}
	const struct pid_config *pid_para = dev->config;
	if (!pid_para->mit) {
		struct pid_single_data *pid_data = data;
		if (pid_data->curr == NULL) {
			return;
		}
		float kp = pid_para->k_p;
		float ki = pid_para->k_i;
		float kd = pid_para->k_d;
		float err = *(pid_data->ref) - *(pid_data->curr);
		float deltaT = k_cyc_to_us_near32(*(pid_data->curr_time) - *(pid_data->prev_time));
		if (!float_equal(ki, 0)) {
			pid_data->err_integral += (err * deltaT) / (1000000 * ki);
		}
		if (!float_equal(kd, 0)) {
			pid_data->err_derivate = kd * err / deltaT;
		}
		//   LOG_INF("integral: %d, derivate: %d", to16t(ki * (err * deltaT) / 1000000),
		//           to16t(kd * 1000000 * err / deltaT));
		*(pid_data->output) = kp * (err + pid_data->err_integral + pid_data->err_derivate);

	} else {
		struct pid_mit_data *pid_data = data;
		if (pid_data->curr == NULL) {
			return;
		}
		float kp = pid_para->k_p;
		float ki = pid_para->k_i;
		float kd = pid_para->k_d;
		float err = *(pid_data->ref) - *(pid_data->curr);
		float deltaT = k_cyc_to_us_near32(*(pid_data->curr_time) - *(pid_data->prev_time));
		if (!float_equal(ki, 0)) {
			pid_data->err_integral += (err * deltaT) / (1000000 * ki);
		}
		if (!float_equal(kd, 0)) {
			pid_data->err_derivate =
				kd * (*(pid_data->detri_ref) - *(pid_data->detri_curr)) / deltaT;
		}

		*(pid_data->output) = kp * (err + pid_data->err_integral + pid_data->err_derivate);
		return;
	}
}

static void pid_register(struct pid_single_data *data, float *curr, float *ref, uint32_t *curr_cyc,
			 uint32_t *prev_cyc, float *output)
{
	if (data == NULL) {
		return;
	}
	data->curr = curr;
	data->ref = ref;
	data->curr_time = curr_cyc;
	data->prev_time = prev_cyc;
	data->output = output;
}

static void mit_register(struct pid_mit_data *data, float *curr, float *ref, float *detri_curr,
			 float *detri_ref, uint32_t *curr_cyc, uint32_t *prev_cyc, float *output)
{
	if (data == NULL) {
		return;
	}
	data->curr = curr;
	data->ref = ref;
	data->detri_curr = detri_curr;
	data->detri_ref = detri_ref;
	data->curr_time = curr_cyc;
	data->prev_time = prev_cyc;
	data->output = output;
}

static const struct pid_single_config *pid_get_params(const struct device *dev)
{
	if (dev == NULL) {
		return NULL;
	}
	return dev->config;
}

#ifdef __cplusplus
}
#endif

#endif // PID_H