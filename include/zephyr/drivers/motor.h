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

#include "zephyr/logging/log_core.h"
#include <zephyr/kernel/thread.h>
#include <errno.h>

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>



#ifdef __cplusplus
extern "C" {
#endif

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

#define MOTOR_TORQUE_CTRL BIT(0)
#define MOTOR_SPEED_CTRL BIT(1)

typedef uint8_t motor_mode_t;


/** @brief Macros for each in dts */
#define MOTOR_NODE DT_NODELABEL(motor)

// #define IS_DJI_COMPAT(node_id) \
//     (DT_NODE_HAS_COMPAT(node_id, dji_m3508) || \
//      DT_NODE_HAS_COMPAT(node_id, dji_m2006) || \
//      DT_NODE_HAS_COMPAT(node_id, dji_m1005) || \
//      DT_NODE_HAS_COMPAT(node_id, dji_others)) // Add other DJI compatible strings as needed

#define GET_DEVICE_POINTER(node_id) DEVICE_DT_GET(node_id),

const struct device *motor_devices[] = {
    DT_FOREACH_CHILD(MOTOR_NODE, GET_DEVICE_POINTER),
};

#define MOTOR_COUNT DT_NUM_INST_STATUS_OKAY(motor)
#define CAN_COUNT DT_NUM_INST_STATUS_OKAY(can)

#define GET_CAN_CHANNEL(node_id) DT_PHANDLE(node_id, can_channel)


/**
 * @brief Motor information structure
 *
 * This structure gathers useful information about motors and neccesary informations.
 */
typedef struct {
	uint8_t expect_current;
	uint8_t expect_rpm;
} motor_ctrl_info_t;

/**
 * @brief Motor information structure
 *
 * This structure gathers useful information about motors and neccesary informations.
 */
typedef struct {
    uint16_t current;
    uint16_t torque;
    uint8_t rpm;
} motor_curr_info_t;



/**
 * @brief Common Motor driver data.
 *
 * This structure is common to all motor drivers and is expected to be the first element in
 * the driver's struct driver_data declaration.
 */
struct motor_driver_data {
	/** Servo Motor label */
	motor_mode_t mode;
	bool started;
	k_tid_t ctrl_thread_tid;
	motor_ctrl_info_t *ctrl_ptr;
	motor_curr_info_t *info_ptr;
};

/**
 * @typedef motor_init()
 * @brief Callback API for initializing motor control mode
 *
 * @see motor_init() for argument descriptions.
 */
typedef int (*motor_api_start)();

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef uint8_t (*motor_api_stat_speed)(const struct device *dev);

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef uint8_t (*motor_api_stat_torque)(const struct device *dev);

/**
 * @typedef motor_set_speed()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int (*motor_api_speed)(const struct device *dev, int16_t speed_rpm, int16_t k_p, int16_t k_i, int16_t k_d);

/**
 * @brief Servo Motor driver API
 */
__subsystem struct motor_driver_api {
	motor_api_start motor_start;
	motor_api_stat_speed motor_get_speed;
	motor_api_stat_torque motor_get_torque;
	motor_api_speed motor_set_speed;
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
 * @brief Init
 *
 * This optional routine starts blinking a LED forever with the given time
 * period.
 *
 * @param dev Motor Device
 * @param ctrl_mode Control mode
 * @return 0 on success, negative on error
 */
__syscall int motor_start();

static inline int z_impl_motor_start()
{
	struct device *cans[CAN_COUNT];
	//fdsafdsafdsa
	for (int i = 0; i < CAN_COUNT; i++) {
        cans[i] = DEVICE_DT_GET(DT_INST(i, can));
        if (device_is_ready(cans[i])) {
            LOG_INF("CAN device %d: %s\n", i, cans[i]->name);
        } else {
            LOG_ERR("CAN device %d not ready\n", i);
        }
    }
	for (int i = 0; i < MOTOR_COUNT; i++) {
        const struct device *dev = motor_devices[i];
        if (dev) {
			for(int j = 0; j < CAN_COUNT; j++) {
				const struct device *can_dev = DEVICE_DT_GET(GET_CAN_CHANNEL(DT_NODELABEL(dev->name)));
				if(can_dev == cans[j])
					dev->data->common->ctrl_ptr = can_dev;
        }
    }
}

/**
 * @brief Get Torque
 *
 * This routine retrieves the current torque of the motor.
 *
 * @param dev Motor Device
 * @return the current torque
 */
__syscall uint8_t motor_get_torque(const struct device *dev);

static inline uint8_t z_impl_motor_get_torque(const struct device *dev)
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
__syscall uint8_t motor_get_speed(const struct device *dev);

static inline uint8_t z_impl_motor_get_speed(const struct device *dev)
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
__syscall int motor_set_speed(const struct device *dev, int16_t speed_rpm, int16_t k_p, int16_t k_i, int16_t k_d);

static inline int z_impl_motor_set_speed(const struct device *dev, int16_t speed_rpm, int16_t k_p, int16_t k_i, int16_t k_d)
{
	const struct motor_driver_api *api =
		(const struct motor_driver_api *)dev->api;

	if (api->motor_init == NULL) {
		return -ENOSYS;
	}
	return api->motor_set_speed(dev, speed_rpm, k_p, k_i, k_d);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif	/* ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_ */