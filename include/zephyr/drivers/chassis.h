/**
 * @file
 * @brief General  Chassis Interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CHASSIS_H_
#define ZEPHYR_INCLUDE_DRIVERS_CHASSIS_H_

/**
 * @brief  Chassis Interface
 * @defgroup chassis_interface  Chassis Interface
 * @since 
 * @version 1.0.0
 * @ingroup interfaces
 * @{
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <errno.h>
#include <sys/_intsup.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <errno.h>
#include <zephyr/devicetree.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
 enum ChassisType{
    MECANUM_WHEEL_CHASSIS,  // 麦轮底盘
    STEERED_WHEEL_CHASSIS,  // 舵轮底盘
    OMNI_WHEEL_CHASSIS      // 全向轮底盘
} ;

struct chassis_driver_config {
    const enum ChassisType type;
    const int              wheel_count;
    const float            wheel_angle;
    const float            wheel_distance;
    const float            wheel_radius;
};

struct chassis_driver_data {
    float vx;
    float vy;
    float omega;
};



/**
 * @brief 底盘接口 API
 */
__subsystem struct chassis_driver_api {
    /**
     * @brief 设置底盘目标位置（用于平移）
     * 
     * @param dev 电机设备指针
     * @param angle 方向
     * @param speed 目标速度
     * @return int 0:成功，负值:错误码
     */
    int (*chassis_set_vel)(const struct device *dev, float vx,float vy, float omega);

  

    /**
     * @brief 设置底盘旋转角度（用于旋转特定角度）
     * 
     * @param dev 电机设备指针
     * @param angle 目标角度（度）
     * @return int 0:成功，负值:错误码
     */
    int (*chassis_set_rotate_angle)(const struct device *dev, float angle);

    int (*chassis_init)(const struct device *dev);
};

/**
 * @brief 设置底盘的目标位置
 *
 * @param dev 
 * @param x 目标x坐标
 * @param y 目标y坐标
 * @param speed 目标速度
 * @return int 0:成功，负值:错误码
 */
__syscall int chassis_set_vel(const struct device *dev, float vx, float vy, float omega);

static inline int z_impl_chassis_set_vel(const struct device *dev, float vx,float vy, float omega) {
    const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;
    if (api->chassis_set_vel == NULL) {
        return -ENOSYS;
    }
    return api->chassis_set_vel(dev, vx,vy, omega);
}



/**
 * @brief 设置底盘的目标旋转角度
 *
 * @param dev 电机设备指针
 * @param angle 目标旋转角度（度）
 * @return int 0:成功，负值:错误码
 */
__syscall int chassis_set_rotate_angle(const struct device *dev, float angle);

static inline int z_impl_chassis_set_rotate_angle(const struct device *dev, float angle) {
    const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;
    if (api->chassis_set_rotate_angle == NULL) {
        return -ENOSYS;
    }
    return api->chassis_set_rotate_angle(dev, angle);
}
__syscall int chassis_init(const struct device *dev);

static inline int z_impl_chassis_init(const struct device *dev) {
    const struct chassis_driver_api *api = (const struct chassis_driver_api *)dev->api;
    if (api->chassis_init == NULL) {
        return -ENOSYS;
    }
    return api->chassis_init(dev);
}


#define CHASSIS_DT_DRIVER_CONFIG_GET(node_id)                                                \
    {                                                                                            \
        .type = DT_PROP(node_id, type),                                                        \
        .wheel_count = DT_PROP(node_id, wheel_count),                                          \
    }

#define CHASSIS_DT_DRIVER_DATA_INST_GET(inst)   {0}
#define CHASSIS_DT_DRIVER_CONFIG_INST_GET(inst) CHASSIS_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/chassis.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_CHASSIS_H_ */
