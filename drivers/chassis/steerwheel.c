/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STEERWHEEL_C
#define STEERWHEEL_C

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/steerwheel.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

#define STEERWHEEL_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)     \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define DT_DRV_COMPAT ares_steerwheel

#define STEERWHEEL_DEVICE_DEFINE(inst)                                                             \
	static steerwheel_data_t steerwheel_data_##inst = {                                        \
		.current_angle = 0.0f,                                                             \
		.current_speed = 0.0f,                                                             \
		.set_angle = 0.0f,                                                                 \
		.negative = false,                                                                 \
	};                                                                                         \
	static const steerwheel_cfg_t steerwheel_cfg_##inst = {                                    \
		.angle_offset = DT_STRING_UNQUOTED(DT_DRV_INST(inst), angle_offset),               \
		.wheel_radius = DT_STRING_UNQUOTED(DT_DRV_INST(inst), wheel_radius),               \
		.steer_motor = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), steer_motor)),             \
		.wheel_motor = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), wheel_motor)),             \
		.inverse_steer = DT_PROP(DT_DRV_INST(inst), inverse_steer),                        \
		.inverse_wheel = DT_PROP(DT_DRV_INST(inst), inverse_wheel),                        \
	};                                                                                         \
	STEERWHEEL_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, &steerwheel_data_##inst,        \
				    &steerwheel_cfg_##inst, POST_KERNEL, 90, NULL);

DT_INST_FOREACH_STATUS_OKAY(STEERWHEEL_DEVICE_DEFINE)

#endif // STEERWHEEL_C