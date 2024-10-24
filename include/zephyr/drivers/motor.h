/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief General Servo Motor Interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

/**
 * @brief Servo Motor Interface
 * @defgroup servo_motor_interface Servo Motor Interface
 * @since 3.7.99
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/kernel.h>
#include "zephyr/logging/log_core.h"
#include <zephyr/kernel/thread.h>
#include <errno.h>
#include <zephyr/sys/util.h>

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <string.h>



#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_TORQUE_CTRL BIT(0)
#define MOTOR_SPEED_CTRL BIT(1)

#define MOTOR_DT_DRIVER_DATA_INST_GET(inst) {0}

#define DT_DRIVER_GET_CANBUS_IDT(node_id) DT_PHANDLE(node_id, can_channel)
#define DT_DRIVER_GET_CANPHY_IDT(node_id) \
		DT_PHANDLE(DT_DRIVER_GET_CANBUS_IDT(node_id), can_device)
#define DT_DRIVER_GET_CANBUS_NAME(node_id) \
		DT_NODE_FULL_NAME(DT_DRIVER_GET_CANBUS_IDT(node_id))

#define DT_GET_CANPHY(node_id) DEVICE_DT_GET(DT_DRIVER_GET_CANPHY_IDT(node_id))
#define DT_GET_CANPHY_BY_BUS(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_device))

#define MOTOR_DT_DRIVER_CONFIG_INST_GET(inst) \
	MOTOR_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))
/**
 * @brief Static initializer for @p motor_driver_config struct
 *
 * @param node_id Devicetree node identifier
 */
#define MOTOR_DT_DRIVER_CONFIG_GET(node_id)				\
	{ \
		.phy = DT_GET_CANPHY(node_id), \
		.canbus = DT_DRIVER_GET_CANBUS_NAME(node_id), \
		.tx_id = DT_PROP(node_id, tx_addr), \
		.rx_id = DT_PROP(node_id, rx_addr), \
		.k_p = DT_PROP(node_id, k_p) / 100.0, \
		.k_i = DT_PROP(node_id, k_i) / 100.0, \
		.k_d = DT_PROP(node_id, k_d) / 100.0, \
		.gear_ratio = (double) DT_PROP(node_id, gear_ratio) / 100.0, \
	}

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)   \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level,	\
			     prio, api, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level,	\
			 prio, api, __VA_ARGS__)

typedef uint8_t motor_mode_t;


struct motor_driver_data {
	/** Motor control mode */
	motor_mode_t mode;
};

struct motor_driver_config {
	/** Physical device */
	const struct device *phy;
	/** CAN Bus device */
	char canbus[12];
	/** CAN TX ID */
	uint32_t tx_id;
	/** CAN RX ID */
	uint32_t rx_id;
	/** Gear Ratio */
	double gear_ratio;
	/** K_P */
	double k_p;
	/** K_I */
	double k_i;
	/** K_D */
	double k_d;
};

/**
 * @typedef motor_init()
 * @brief Callback API for initializing motor control mode
 *
 * @see motor_init() for argument descriptions.
 */
// typedef int (*motor_api_start)();

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int16_t (*motor_api_stat_speed_t)(const struct device *dev);

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int16_t (*motor_api_stat_torque_t)(const struct device *dev);

/**
 * @typedef motor_set_speed()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int8_t (*motor_api_speed_t)(const struct device *dev, int16_t speed_rpm);

/**
 * @brief Servo Motor driver API
 */
__subsystem struct motor_driver_api {
	motor_api_stat_speed_t motor_get_speed;
	motor_api_stat_torque_t motor_get_torque;
	motor_api_speed_t motor_set_speed;
};

/**
 * @brief Motor error codes
 */
enum MotorError {
    MOTOR_SUCCESS = 0,
    MOTOR_ERROR_INIT = -1,
    MOTOR_ERROR_COMM = -2,
    MOTOR_ERROR_UNKNOWN = -3
};


/**
 * @brief Get Torque
 *
 * This routine retrieves the current torque of the motor.
 *
 * @param dev Motor Device
 * @return the current torque
 */
__syscall int16_t motor_get_torque(const struct device *dev);

static inline int16_t z_impl_motor_get_torque(const struct device *dev)
{
	const struct motor_driver_api *api =
		(const struct motor_driver_api *)dev->api;

	if (api->motor_get_torque == NULL) {
		return -ENOSYS;
	}
	return api->motor_get_torque(dev);
}

/**
 * @brief Get Speed
 *
 * This optional routine retrieves the current speed of the motor.
 *
 * @param dev Motor Device
 * @return the current speed in RPM
 */
__syscall int16_t motor_get_speed(const struct device *dev);

static inline int16_t z_impl_motor_get_speed(const struct device *dev)
{
	const struct motor_driver_api *api =
		(const struct motor_driver_api *)dev->api;

	if (api->motor_get_speed == NULL) {
		return -ENOSYS;
	}
	return api->motor_get_speed(dev);
}

/**
 * @brief Get Status
 *
 * This optional routine starts blinking a LED forever with the given time
 * period.
 *
 * @param dev Motor Device
 * @return 0 on success, negative on error
 */
__syscall int8_t motor_set_speed(const struct device *dev, int16_t speed_rpm);

static inline int8_t z_impl_motor_set_speed(const struct device *dev, int16_t speed_rpm)
{
	const struct motor_driver_api *api =
		(const struct motor_driver_api *)dev->api;

	if (api->motor_set_speed == NULL) {
		return -ENOSYS;
	}
	return api->motor_set_speed(dev, speed_rpm);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif	/* ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_ */