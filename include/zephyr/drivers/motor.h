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

/**
 * @brief 电机工作模式枚举
 * 
 * MIT: MIT模式
 * PV: 位置-速度控制
 * VO: 速度控制  
 * MULTILOOP: 多环串联控制
 */
enum motor_mode { MIT, PV, VO, MULTILOOP };

/**
 * @brief 电机控制命令枚举
 */
enum motor_cmd { ENABLE_MOTOR, DISABLE_MOTOR, SET_ZERO_OFFSET, CLEAR_PID, CLEAR_ERROR };

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
    const struct device *controller[4];
    /** Motor capabilities */
    char capabilities[4][12];
};

struct motor_driver_data {
    float angle;
    float rpm;
    float torque;
    float temperature;
    int   round_cnt;

    enum motor_mode mode;
};
/**
 * @typedef motor_api_stat_speed_t
 * @brief 获取电机当前速度的回调函数
 *
 * @param dev 指向电机设备的指针
 * @return float 当前电机速度(rpm)
 */
typedef float (*motor_api_stat_speed_t)(const struct device *dev);

/**
 * @typedef motor_api_stat_torque_t  
 * @brief 获取电机当前扭矩的回调函数
 *
 * @param dev 指向电机设备的指针
 * @return float 当前电机扭矩(N·m)
 */
typedef float (*motor_api_stat_torque_t)(const struct device *dev);

/**
 * @typedef motor_api_stat_angle_t
 * @brief 获取电机当前角度的回调函数
 *
 * @param dev 指向电机设备的指针
 * @return float 当前电机角度(度)
 */
typedef float (*motor_api_stat_angle_t)(const struct device *dev);

/**
 * @typedef motor_api_speed_t
 * @brief 设置电机目标速度的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param speed_rpm 目标速度(rpm)
 * @return int 0表示成功,负值表示错误码
 */
typedef int (*motor_api_speed_t)(const struct device *dev, float speed_rpm);

/**
 * @typedef motor_api_torque_t
 * @brief 设置电机目标扭矩的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param torque 目标扭矩(N·m)
 * @return int 0表示成功,负值表示错误码
 */
typedef int (*motor_api_torque_t)(const struct device *dev, float torque);

/**
 * @typedef motor_api_angle_t
 * @brief 设置电机目标角度的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param angle 目标角度(度)
 * @return int 0表示成功,负值表示错误码
 */
typedef int (*motor_api_angle_t)(const struct device *dev, float angle);

/**
 * @typedef motor_api_ctrl_t
 * @brief 电机控制命令的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param cmd 控制命令(启动、停止等)
 * @return void
 */
typedef void (*motor_api_ctrl_t)(const struct device *dev, enum motor_cmd cmd);

/**
 * @typedef motor_api_mode_t
 * @brief 设置电机工作模式的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param mode 工作模式(速度模式、位置模式等)
 * @return int 0表示成功,负值表示错误码
 */
typedef int (*motor_api_mode_t)(const struct device *dev, enum motor_mode mode);
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
    motor_api_mode_t        motor_set_mode;
    motor_api_ctrl_t        motor_control;
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
 * @brief 获取电机当前扭矩
 *
 * @param dev 电机设备指针
 * @return float 当前扭矩值(N·m)，错误时返回负值
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
 * @brief 获取电机当前转速
 *
 * @param dev 电机设备指针
 * @return float 当前转速(RPM)，错误时返回负值
 */
__syscall float motor_get_speed(const struct device *dev);

static inline float z_impl_motor_get_speed(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_get_speed == NULL) {
        return -ENOSYS;
    }
    return api->motor_get_speed(dev);
}

/**
 * @brief 获取电机当前角度
 *
 * @param dev 电机设备指针
 * @return float 当前角度(度)，错误时返回负值
 */
__syscall float motor_get_angle(const struct device *dev);

static inline float z_impl_motor_get_angle(const struct device *dev) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_get_angle == NULL) {
        return -ENOSYS;
    }
    return api->motor_get_angle(dev);
}

/**
 * @brief 设置电机目标速度
 *
 * @param dev 电机设备指针
 * @param speed_rpm 目标转速(RPM)
 * @return int 0:成功，负值:错误码
 */
__syscall int motor_set_speed(const struct device *dev, float speed_rpm);

static inline int z_impl_motor_set_speed(const struct device *dev, float speed_rpm) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_set_speed == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_speed(dev, speed_rpm);
}

/**
 * @brief 设置电机目标角度
 *
 * @param dev 电机设备指针
 * @param angle 目标角度(度)
 * @return int 0:成功，负值:错误码
 */
__syscall int motor_set_angle(const struct device *dev, float angle);

static inline int z_impl_motor_set_angle(const struct device *dev, float angle) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_set_angle == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_angle(dev, angle);
}

/**
 * @brief 设置电机目标扭矩
 *
 * @param dev 电机设备指针
 * @param torque 目标扭矩(N·m)
 * @return int 0:成功，负值:错误码
 */
__syscall int motor_set_torque(const struct device *dev, float torque);

static inline int z_impl_motor_set_torque(const struct device *dev, float torque) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_set_torque == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_torque(dev, torque);
}

/**
 * @brief 执行电机控制命令
 *
 * @param dev 电机设备指针
 * @param cmd 控制命令
 * @return int 0:成功，负值:错误码
 */
__syscall void motor_control(const struct device *dev, enum motor_cmd cmd);

static inline void z_impl_motor_control(const struct device *dev, enum motor_cmd cmd) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_control == NULL) {
        return;
    }
    api->motor_control(dev, cmd);
    return;
}

/**
 * @brief 设置电机工作模式
 *
 * @param dev 电机设备指针
 * @param mode 工作模式
 * @return int 0:成功，负值:错误码
 */
__syscall int motor_set_mode(const struct device *dev, enum motor_mode mode);

static inline int z_impl_motor_set_mode(const struct device *dev, enum motor_mode mode) {
    const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
    if (api->motor_set_mode == NULL) {
        return -ENOSYS;
    }
    return api->motor_set_mode(dev, mode);
}

#define CAN_COUNT DT_NUM_INST_STATUS_OKAY(vnd_canbus)

#define DT_DRIVER_GET_CANBUS_IDT(node_id) DT_PHANDLE(node_id, can_channel)
#define DT_DRIVER_GET_CANPHY_IDT(node_id)                                                        \
    DT_PHANDLE(DT_DRIVER_GET_CANBUS_IDT(node_id), can_device)

#define DT_GET_CANPHY(node_id) DEVICE_DT_GET(DT_DRIVER_GET_CANPHY_IDT(node_id))

#define DT_GET_CANPHY_BY_BUS(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_device))

#define GET_CONTROLLER_STRUCT(node_id, prop, idx)                                                \
    DEVICE_DT_GET(DT_PROP_BY_IDX(node_id, prop, idx))

#define MOTOR_DT_DRIVER_CONFIG_GET(node_id)                                                      \
    {                                                                                            \
        .phy          = (const struct device *)DT_GET_CANPHY(node_id),                           \
        .id           = DT_PROP(node_id, id),                                                    \
        .tx_id        = DT_PROP(node_id, tx_id),                                                 \
        .rx_id        = DT_PROP(node_id, rx_id),                                                 \
        .controller   = {DT_FOREACH_PROP_ELEM_SEP(node_id, controllers, GET_CONTROLLER_STRUCT,   \
                                                  (, ))},                                        \
        .capabilities = DT_PROP(node_id, capabilities),                                          \
    }

#define MOTOR_DT_DRIVER_DATA_INST_GET(inst)   {0}
#define MOTOR_DT_DRIVER_CONFIG_INST_GET(inst) MOTOR_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_ */