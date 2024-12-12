/*
 * motor_dm.h
 *
 * Header file for motor_dm.c
 */

#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#define _USE_MATH_DEFINES
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>
#include <math.h>

/*  canbus_id_t specifies the ID of the CAN bus the motor is on
        which is defined in motor_devices[] */
typedef uint8_t canbus_id_t;

/*  allmotor_id_t specifies the ID of the motor
        which is in the flags
        use flags |= 1 << all_motor_id*/
typedef uint16_t allmotor_id_t;

/*  motor_id_t specifies the ID of the motor in motor_controller struct
        find the rpm in motor_cans->target_rpm[canbus_id] */
typedef uint16_t motor_id_t;

#define INT12_MAX 0x7FF
#define INT12_MIN -0x800

#define PI 3.14159265f

struct motor_controller {
    const struct device *can_dev;

    int            ids[8];
    bool           full[5];
    uint8_t        mapping[5][4];
    uint8_t        flags;
    struct device *motor_devs[8];
    struct k_sem   thread_sem;
};

const float RAD2ROUND          = 1.0f / (2 * PI);
const float DM_J4310_2EC_V_MAX = 30.0f;
const float DM_J4310_2EC_P_MAX = 12.5f;
const float DM_J4310_2EC_T_MAX = 10.0f;

const float DM_J4310_2EC_KP_MIN = 0.0f;
const float DM_J4310_2EC_KP_MAX = 500.0f;
const float DM_J4310_2EC_KD_MIN = 0.0f;
const float DM_J4310_2EC_KD_MAX = 5.0f;

struct dm_motor_data {
    struct motor_driver_data common;
    canbus_id_t              canbus_id;
    struct motor_controller *ctrl_struct;

    // Control status
    bool            online;
    uint8_t         convert_num;
    int8_t          current_mode_index;
    enum motor_mode mode;

    // RAW DATA
    uint16_t RAWangle;
    uint16_t RAWprev_angle;
    int32_t  RAWtorque;
    int16_t  RAWrpm;
    int8_t   RAWtemp_mos;
    int8_t   RAWtemp_rotor;
    uint32_t curr_time;
    uint32_t prev_time;
    int32_t  RAWangle_add;
    int8_t   missed_times;
    int8_t   err;

    // Target
    float target_angle;
    float target_rpm;
    float target_torque;

    struct pid_single_config params;
};

struct dm_motor_config {
    struct motor_driver_config common;

    float gear_ratio;
    bool  is_gm6020;
    bool  is_m3508;
    bool  is_m2006;

    float v_max;
    float p_max;
    float t_max;
};

// 全局变量声明
extern struct motor_controller motor_cans[];

// 函数声明
void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data);

// 如果需要在其他文件中使用以下函数，请取消对应函数定义中的 static 关键字
int8_t dm_set_speed(const struct device *dev, float speed_rpm);
int8_t dm_set_angle(const struct device *dev, float angle);
int8_t dm_set_torque(const struct device *dev, float torque);
float  dm_set_zero(const struct device *dev);
float  dm_get_angle(const struct device *dev);
float  dm_get_speed(const struct device *dev);
float  dm_get_torque(const struct device *dev);
int    dm_init(const struct device *dev);

static const struct motor_driver_api motor_api_funcs = {
    .motor_get_speed  = dm_get_speed,
    .motor_get_torque = dm_get_torque,
    .motor_get_angle  = dm_get_angle,
    .motor_set_speed  = dm_set_speed,
    .motor_set_torque = dm_set_torque,
    .motor_set_angle  = dm_set_angle,
    .motor_set_zero   = dm_set_zero,
};

extern const struct device *can_devices[];
extern const struct device *motor_devices[];

#define CAN_COUNT DT_NUM_INST_STATUS_OKAY(vnd_canbus)

#endif // MOTOR_dm_H