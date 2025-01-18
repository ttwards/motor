#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/chassis.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
const struct device *can_dev = DEVICE_DT_GET(DT_INST(0 ,vnd_canbus));
const struct device *motor1  = DEVICE_DT_GET(DT_NODELABEL(motor0));
const struct device *acc     = DEVICE_DT_GET(DT_INST(0, bosch_bmi08x_accel));
const struct device *gyro =DEVICE_DT_GET(DT_INST(0 ,bosch_bmi08x_gyro));
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
const struct device *steer_chassis = DEVICE_DT_GET(DT_INST(0, steer_chassis));
int main(void)
{
    
 
    
    
    k_sleep(K_MSEC(50));
    
    motor_set_mode(motor1, MIT);
    motor_control(motor1, ENABLE_MOTOR);
    motor_set_speed(motor1, 20);

    
   
   

    while (1) {
        // motor1_rpm = motor_get_speed(motor1);
        // LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm, (double)motor_get_speed(motor1));
        k_msleep(5);
        // k_sleep(K_FOREVER);
    }
}