/* steered_chassis.c */
#include "chassis_steer.h"
#include <zephyr/drivers/chassis.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <math.h>
#define DT_DRV_COMPAT steered_chassis

static void normalize_angle(float *angle) {
    while (*angle > PI) *angle -= 2 * PI;
    while (*angle < -PI) *angle += 2 * PI;
}

static float get_optimal_rotation_direction(float current, float target) {
    float diff = target - current;
    normalize_angle(&diff);
    
    if (diff > PI) {
        return diff - 2 * PI;
    } else if (diff < -PI) {
        return diff + 2 * PI;
    }
    return diff;
}

static void calculate_wheel_velocities(const struct steered_chassis_config *config,
                                    struct wheel_state *wheel,
                                    float vx, float vy, float omega,
                                    int wheel_idx) {
    float wheel_angle = config->common.wheel_angle + (2 * PI * wheel_idx) / config->common.wheel_count;
    float wheel_x = ((double)config->common.wheel_distance * cos((double)wheel_angle));
    float wheel_y = (double)config->common.wheel_distance * sin((double)wheel_angle);
    
    float v_tangential = -omega * wheel_y;
    float v_radial = omega * wheel_x;
    float v_total_x = vx + v_tangential;
    float v_total_y = vy + v_radial;
    
    wheel->target_angle = atan2(v_total_y, v_total_x);
    wheel->target_speed = sqrt(v_total_x * v_total_x + v_total_y * v_total_y);
    
    float angle_diff = get_optimal_rotation_direction(wheel->current_angle, wheel->target_angle);
    
    if (fabs(angle_diff) > PI/2) {
        wheel->target_speed = -wheel->target_speed;
        wheel->target_angle += PI;
        normalize_angle(&wheel->target_angle);
    }
}

static void update_thread_func(void *dev_ptr, void *unused1, void *unused2) {
    const struct device *dev = (const struct device *)dev_ptr;
    struct steered_chassis_data *data = dev->data;
    const struct steered_chassis_config *config = dev->config;
    
    while (data->running) {
        k_mutex_lock(&data->state_mutex, K_FOREVER);
        
        for (int i = 0; i < config->common.wheel_count; i++) {
            struct wheel_state *wheel = &data->wheels[i];
            
            float angle_diff = get_optimal_rotation_direction(wheel->current_angle, 
                                                           wheel->target_angle);
            
            // 电机控制实现
            if (wheel->steering_motor) {
                motor_set_angle(wheel->steering_motor, wheel->target_angle);
            }
            
            if (wheel->drive_motor) {
                motor_set_speed(wheel->drive_motor, wheel->target_speed);
            }
            
            wheel->current_angle += angle_diff * 0.1f;
            normalize_angle(&wheel->current_angle);
            wheel->current_speed = wheel->target_speed;
        }
        
        k_mutex_unlock(&data->state_mutex);
        k_msleep(1000 / data->update_rate_hz);  // 使用配置的更新频率
    }
}

// 设置更新频率
int steered_chassis_set_update_rate(const struct device *dev, uint32_t rate_hz) {
    struct steered_chassis_data *data = dev->data;
    
    if (rate_hz == 0) {
        return -EINVAL;
    }
    
    k_mutex_lock(&data->state_mutex, K_FOREVER);
    data->update_rate_hz = rate_hz;
    k_mutex_unlock(&data->state_mutex);
    
    return 0;
}

int steered_chassis_init(const struct device *dev) {
    struct steered_chassis_data *data = dev->data;
    const struct steered_chassis_config *config = dev->config;
    
    // 从设备树获取轮子数量
    int wheel_count = config->common.wheel_count;
    
    // 动态分配轮子状态数组
    data->wheels = k_malloc(sizeof(struct wheel_state) * wheel_count);
    if (!data->wheels) {
        return -ENOMEM;
    }
    
    k_mutex_init(&data->state_mutex);
    data->running = true;
    data->update_rate_hz = DEFAULT_UPDATE_RATE_HZ;
    
    // 初始化轮子状态
    for (int i = 0; i < wheel_count; i++) {
        data->wheels[i].current_angle = 0.0f;
        data->wheels[i].target_angle = 0.0f;
        data->wheels[i].current_speed = 0.0f;
        data->wheels[i].target_speed = 0.0f;
        
        // 获取电机设备
        char drive_motor_name[32];
        char steer_motor_name[32];
        snprintf(drive_motor_name, sizeof(drive_motor_name), "drive_motor_%d", i);
        snprintf(steer_motor_name, sizeof(steer_motor_name), "steer_motor_%d", i);
        
        data->wheels[i].drive_motor = device_get_binding(drive_motor_name);
        data->wheels[i].steering_motor = device_get_binding(steer_motor_name);
        
        if (!data->wheels[i].drive_motor || !data->wheels[i].steering_motor) {
            k_free(data->wheels);
            return -ENODEV;
        }
    }
    
    // 启动更新线程
    k_thread_create(&data->update_thread, data->thread_stack,
                   K_THREAD_STACK_SIZEOF(data->thread_stack),
                   update_thread_func, (void *)dev, NULL, NULL,
                   CHASSIS_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    return 0;
}

// 其他函数保持不变...

/* 设备定义宏 */
#define STEERED_CHASSIS_INIT(inst) \
    static struct steered_chassis_data steered_data_##inst = { \
        .common = { \
            .vx = 0.0f, \
            .vy = 0.0f, \
            .omega = 0.0f, \
        }, \
    }; \
    static const struct steered_chassis_config steered_config_##inst = { \
        .common = { \
            .type = STEERED_WHEEL_CHASSIS, \
            .wheel_count = DT_INST_PROP(inst, wheel_count), \
            .wheel_angle = DT_INST_PROP(inst, wheel_angle), \
            .wheel_distance = DT_INST_PROP(inst, wheel_distance), \
            .wheel_radius = DT_INST_PROP(inst, wheel_radius), \
        }, \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
                         steered_chassis_init, \
                         NULL, \
                         &steered_data_##inst, \
                         &steered_config_##inst, \
                         POST_KERNEL, \
                         CONFIG_CHASSIS_INIT_PRIORITY, \
                         &steered_chassis_api);

DT_INST_FOREACH_STATUS_OKAY(STEERED_CHASSIS_INIT)
