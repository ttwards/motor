/* steered_chassis.h */
#ifndef STEERED_CHASSIS_H
#define STEERED_CHASSIS_H

#include <zephyr/drivers/chassis.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <math.h>

// 可配置参数
#define CHASSIS_THREAD_PRIORITY 5
#define CHASSIS_STACK_SIZE 1024
#define DEFAULT_UPDATE_RATE_HZ 100
#define PI 3.141592653589f
struct wheel_state {
    float current_angle;    
    float target_angle;     
    float current_speed;    
    float target_speed;     
    const struct device *drive_motor;
    const struct device *steering_motor;
};

struct steered_chassis_data {
    struct chassis_driver_data common;
    struct wheel_state *wheels;  // 动态分配轮子数组
    struct k_mutex state_mutex;
    struct k_thread update_thread;
    bool running;
    uint32_t update_rate_hz;  // 可配置的更新频率
    k_thread_stack_t thread_stack[CHASSIS_STACK_SIZE];
};

struct steered_chassis_config {
    struct chassis_driver_config common;
    const struct device **wheel_devices;  // 动态分配设备数组
};

// 初始化和配置函数
int steered_chassis_init(const struct device *dev);
int steered_chassis_set_update_rate(const struct device *dev, uint32_t rate_hz);

// 控制函数
int steered_chassis_set_vel(const struct device *dev, float vx, float vy, float omega);
int steered_chassis_set_rotate_angle(const struct device *dev, float angle);

#endif /* STEERED_CHASSIS_H */