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

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)                                                   \
	{                                                                                          \
		.k_p = DT_STRING_UNQUOTED(node_id, k_p),                                           \
		.k_i = DT_STRING_UNQUOTED(node_id, k_i),                                           \
		.k_d = DT_STRING_UNQUOTED(node_id, k_d),                                           \
		.mit = false,                                                                      \
	}

#define PID_CONFIG_DEFINE(inst)                                                                    \
	static const struct pid_config pid_single_config_##inst =                                  \
		PID_SINGLE_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst));

#define PID_DATA_DEFINE(inst)                                                                      \
	static struct pid_single_data pid_single_data_##inst = {.err_integral = 0,                 \
								.err_derivate = 0,                 \
								.curr = NULL,                      \
								.ref = NULL,                       \
								.output = NULL,                    \
								.curr_time = NULL,                 \
								.prev_time = NULL};

#define PID_INST(inst)                                                                             \
	PID_CONFIG_DEFINE(inst)                                                                    \
	PID_DATA_DEFINE(inst)                                                                      \
	PID_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, NULL, &pid_single_config_##inst,       \
			     POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, &pid_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

#endif // PID_SINGLE_H