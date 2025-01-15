/*
 * motor_dji.h
 *
 * Header file for motor_dji.c
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include <stdbool.h>
#include <stdint.h>
#include "dji_ratios.h"
#include "dji_macros.h"
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>

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

struct motor_controller {
    const struct device *can_dev;

    /*
      There are 4 tx addresses
      0x1FF, 0x200 for M3508/M2006
      0x1FE, 0x2FE for GM6020 Current control.
      !!!! Voltage control for GM6020 is deprecated !!!!
      The 5 nums in full[] are: 0x200, 0x1FF, 0x1FE, 0x2FE, 0x2FF
  */
    int            rx_ids[8];
    bool           full[5];
    uint8_t        mapping[5][4];
    uint8_t        flags;
    uint8_t        mask[5];
    struct device *motor_devs[8];
    struct k_sem   thread_sem;
};

struct dji_motor_data {
    struct motor_driver_data common;
    canbus_id_t              canbus_id;
    struct motor_controller *ctrl_struct;

    // Control status
    bool    online;
    uint8_t convert_num;
    int8_t  current_mode_index;

    // RAW DATA
    uint16_t RAWangle;
    uint16_t RAWprev_angle;
    int32_t  RAWcurrent;
    int16_t  RAWrpm;
    int8_t   RAWtemp;
    uint32_t curr_time;
    uint32_t prev_time;
    int32_t  RAWangle_add;
    int8_t   missed_times;

    // Target
    float target_angle;
    float target_rpm;
    float target_torque;
    float target_current;
};

struct dji_motor_config {
    struct motor_driver_config common;

    float gear_ratio;
    bool  is_gm6020;
    bool  is_m3508;
    bool  is_m2006;
};

// 全局变量声明
extern struct motor_controller ctrl_structs[];

struct k_sem tx_queue_sem[CAN_COUNT];

// 函数声明
void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data);

int   dji_set_speed(const struct device *dev, float speed_rpm);
int   dji_set_angle(const struct device *dev, float angle);
int   dji_set_torque(const struct device *dev, float torque);
float dji_set_zero(const struct device *dev);
float dji_get_angle(const struct device *dev);
float dji_get_speed(const struct device *dev);
float dji_get_torque(const struct device *dev);
int   dji_init(const struct device *dev);
void  dji_control(const struct device *dev, enum motor_cmd cmd);

static const struct motor_driver_api motor_api_funcs = {
    .motor_get_speed  = dji_get_speed,
    .motor_get_torque = dji_get_torque,
    .motor_get_angle  = dji_get_angle,
    .motor_set_speed  = dji_set_speed,
    .motor_set_torque = dji_set_torque,
    .motor_set_angle  = dji_set_angle,
    .motor_control    = dji_control,
};

extern const struct device *can_devices[];
extern const struct device *motor_devices[];

#endif // MOTOR_DJI_H