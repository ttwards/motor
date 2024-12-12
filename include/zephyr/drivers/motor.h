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

#include <sys/_intsup.h>
#include <zephyr/kernel.h>
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

enum motor_mode { MIT, PV, VO, MULTILOOP };

struct motor_driver_config {
    /** Physical device */
    const struct device *phy;
    /** motor ID  */
    uint8_t id;
    /** CAN TX ID */
    int tx_id;
    /** CAN RX ID */
    int rx_id;
    /** Gear Ratio */
    struct device *controller[4];
    /** Motor capabilities */
    char capabilities[4][12];
};

struct motor_driver_data {
    float           angle;
    float           rpm;
    float           torque;
    float           temperature;
    enum motor_mode mode;
};

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef float (*motor_api_stat_speed_t)(const struct device *dev);

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef float (*motor_api_stat_torque_t)(const struct device *dev);

/**
 * @typedef motor_get_status()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef float (*motor_api_stat_angle_t)(const struct device *dev);

/**
 * @typedef motor_set_speed()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int8_t (*motor_api_speed_t)(const struct device *dev, float speed_rpm);

/**
 * @typedef motor_set_torque()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int8_t (*motor_api_torque_t)(const struct device *dev, float torque);

/**
 * @typedef motor_set_angle()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef int8_t (*motor_api_angle_t)(const struct device *dev, float angle);

/**
 * @typedef motor_set_zero()
 * @brief Callback API returning motor status
 *
 * @see get_status() for argument descriptions.
 */
typedef float (*motor_api_zero_t)(const struct device *dev);

/**
 * @brief Servo Motor driver API
 */
__subsystem struct motor_driver_api {
    motor_api_stat_speed_t  motor_get_speed;
    motor_api_stat_torque_t motor_get_torque;
    motor_api_stat_torque_t motor_get_angle;
    motor_api_speed_t       motor_set_speed;
    motor_api_torque_t      motor_set_torque;
    motor_api_angle_t       motor_set_angle;
    motor_api_zero_t        motor_set_zero;
};

/**
 * @brief Motor error codes
 */
enum MotorError {
    MOTOR_SUCCESS       = 0,
    MOTOR_ERROR_INIT    = -1,
    MOTOR_ERROR_COMM    = -2,
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
__syscall float motor_get_torque(const struct device *dev);

static inline float z_impl_motor_get_torque(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

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
__syscall float motor_get_speed(const struct device *dev);

static inline float z_impl_motor_get_speed(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_get_speed == NULL) {
        return -ENOSYS;
    }
    return api->motor_get_speed(dev);
}

__syscall float motor_get_angle(const struct device *dev);

static inline float z_impl_motor_get_angle(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_get_angle == NULL) {
        return -ENOSYS;
    }
    return api->motor_get_angle(dev);
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
__syscall int8_t motor_set_speed(const struct device *dev, float speed_rpm);

static inline int8_t z_impl_motor_set_speed(const struct device *dev, float speed_rpm) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_set_speed == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_speed(dev, speed_rpm);
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
__syscall int8_t motor_set_angle(const struct device *dev, float angle);

static inline int8_t z_impl_motor_set_angle(const struct device *dev, float angle) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_set_angle == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_angle(dev, angle);
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
__syscall int8_t motor_set_torque(const struct device *dev, float torque);

static inline int8_t z_impl_motor_set_torque(const struct device *dev, float torque) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_set_torque == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_torque(dev, torque);
}

/**
 * @brief Set Zero
 * 
 * This optional routine sets the current angle of the motor to zero.
 *
 * @param dev Motor Device
    * @return angle before being set to zero
 */
__syscall float     motor_set_zero(const struct device *dev);
static inline float z_impl_motor_set_zero(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;

    if (api->motor_set_zero == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_zero(dev);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_ */