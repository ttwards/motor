/*
 * motor_dji.h
 *
 * Header file for motor_dji.c
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include <stdbool.h>
#include <stdint.h>
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

// 结构体定义
struct motor_info {
  uint16_t angle;
  uint16_t prev_angle;
  int16_t current;
  int16_t rpm;
  float PCD_angle;
  float PCD_rpm;
  float PCD_torque;
  int16_t temperature;
  uint32_t curr_time;
  uint32_t prev_time;
  float angle_offset;
  int32_t RAW_angle;
};

struct motor_controller {
  const struct device *can_dev;
  float target_angle[8];
  float target_rpm[8];
  float target_torque[8];
  int16_t target_current[8];
  /*
      There are 5 tx addresses
      0x1FF, 0x200 for M3508/M2006
      0x1FF, 0x2FF for GM6020 Voltage control
      0x1FE, 0x2FE for GM6020 Current control.
      !!!! Current control for GM6020 is deprecated !!!!
      The 3 nums in tx_enabled[] are: 0x200, 0x1FF, 0x2FF.
  */
  int rx_ids[8];
  bool full[3];
  uint8_t mapping[3][4];
  uint8_t flags;
  uint8_t mask[3];
  struct device *motor_devs[8];
  struct motor_info info[8];
};

struct dji_motor_data {
  struct motor_driver_data common;
  canbus_id_t canbus_id;
  uint8_t current_mode_index;
  struct motor_controller *ctrl_struct;
  uint8_t convert_num;
};

struct dji_motor_config {
  struct motor_driver_config common;
  float gear_ratio;
  float current_ratio;
};

// 全局变量声明
extern struct motor_controller motor_cans[];

// 函数声明
int frames_id(int tx_id);
void can_rx_callback(const struct device *can_dev, struct can_frame *frame,
                     void *user_data);

// 如果需要在其他文件中使用以下函数，请取消对应函数定义中的 static 关键字
int8_t dji_set_speed(const struct device *dev, float speed_rpm);
int8_t dji_set_angle(const struct device *dev, float angle);
int8_t dji_set_torque(const struct device *dev, float torque);
void dji_set_zero(const struct device *dev);
float dji_get_angle(const struct device *dev);
float dji_get_speed(const struct device *dev);
float dji_get_torque(const struct device *dev);
int dji_init(const struct device *dev);

#endif // MOTOR_DJI_H