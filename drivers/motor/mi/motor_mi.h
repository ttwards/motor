#ifndef MOTOR_MI_H
#define MOTOR_MI_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>

//控制参数最值
#define P_MIN  -12.5f
#define P_MAX  12.5f
#define V_MIN  -30.0f
#define V_MAX  30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN  -12.0f
#define T_MAX  12.0f
#define MAX_P  720
#define MIN_P  -720

//主机CAN ID设置
#define Master_CAN_ID 0x00

//控制命令宏定义
#define Communication_Type_GetID              0x00 //获取设备ID和64位MCU唯一标识符
#define Communication_Type_MotionControl      0x01 //向主机发送控制指令
#define Communication_Type_MotorRequest       0x02 //向主机反馈电机运行状态
#define Communication_Type_MotorEnable        0x03 //电机使能运行
#define Communication_Type_MotorStop          0x04 //电机停止运行
#define Communication_Type_SetPosZero         0x06 //设置电机机械零位
#define Communication_Type_CanID              0x07 //更改当前电机CAN_ID
#define Communication_Type_Control_Mode       0x12
#define Communication_Type_GetSingleParameter 0x11 //读取单个参数
#define Communication_Type_SetSingleParameter 0x12 //设定单个参数
#define Communication_Type_ErrorFeedback      0x15 //故障反馈帧

// 操作宏定义
#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

//参数读取宏定义
#define Run_mode      0x7005
#define Iq_Ref        0x7006
#define Spd_Ref       0x700A
#define Limit_Torque  0x700B
#define Cur_Kp        0x7010
#define Cur_Ki        0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref       0x7016
#define Limit_Spd     0x7017
#define Limit_Cur     0x7018

#define Gain_Angle  720 / 32767.0
#define Bias_Angle  0x8000
#define Gain_Speed  30 / 32767.0
#define Bias_Speed  0x8000
#define Gain_Torque 12 / 32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK    0x01

enum CONTROL_MODE //控制模式定义
{
    Motion_mode = 0, //运控模式
    Position_mode,   //位置模式
    Speed_mode,      //速度模式
    Current_mode     //电流模式
};

enum ERROR_TAG //错误回传对照
{
    OK                 = 0, //无故障
    BAT_LOW_ERR        = 1, //欠压故障
    OVER_CURRENT_ERR   = 2, //过流
    OVER_TEMP_ERR      = 3, //过温
    MAGNETIC_ERR       = 4, //磁编码故障
    HALL_ERR_ERR       = 5, //HALL编码故障
    NO_CALIBRATION_ERR = 6  //未标定
};

typedef struct {
    struct motor_driver_data common;
    //txid: Master_CAN_ID << 8 | Motor->CAN_ID

    uint8_t CAN_ID; //CAN ID
    uint8_t MCU_ID; //MCU唯一标识符[后8位，共64位]
    float   Angle;  //回传角度
    float   Speed;  //回传速度
    float   Torque; //回传力矩
    float   Temp;   //回传温度

    uint16_t set_current;
    uint16_t set_speed;
    uint16_t set_position;

    uint8_t error_code;

    float Angle_Bias;
} mi_motor_data;

typedef struct {
    struct motor_driver_config common;

} mi_motor_cfg;

extern mi_motor_data mi_motor[4]; //预定义四个小米电机

extern void check_cybergear(uint8_t ID);
extern void start_cybergear(mi_motor_data *Motor);
extern void stop_cybergear(mi_motor_data *Motor, uint8_t clear_error);
extern void set_mode_cybergear(mi_motor_data *Motor, uint8_t Mode);
extern void set_current_cybergear(mi_motor_data *Motor, float Current);
extern void set_zeropos_cybergear(mi_motor_data *Motor);
extern void set_CANID_cybergear(mi_motor_data *Motor, uint8_t Can_ID);
extern void init_cybergear(mi_motor_data *Motor, uint8_t Can_ID, uint8_t mode);
extern void motor_controlmode(mi_motor_data *Motor, float torque, float MechPosition, float speed,
                              float kp, float kd);

#endif