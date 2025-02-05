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

#define PID_SINGLE_DT_DRIVER_CONFIG_GET(node_id)                                                   \
	{                                                                                          \
		.k_p = DT_STRING_UNQUOTED(node_id, k_p),                                           \
		.k_i = DT_STRING_UNQUOTED(node_id, k_i),                                           \
		.k_d = DT_STRING_UNQUOTED(node_id, k_d),                                           \
		.integral_limit = DT_STRING_UNQUOTED_OR(node_id, i_max, 0),                        \
		.output_limit = DT_STRING_UNQUOTED_OR(node_id, out_max, 0),                        \
	}

#define PID_CONFIG_DEFINE(inst)                                                                    \
	static const struct pid_config mit_cfg_##inst = {                                          \
		.common = PID_SINGLE_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))};

#define PID_INST(inst)                                                                             \
	PID_CONFIG_DEFINE(inst)                                                                    \
	PID_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, NULL, &mit_cfg_##inst, POST_KERNEL,    \
			     CONFIG_MOTOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

#endif // PID_MIT_H