#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
// 获取各个引脚的节点
// #define POWER_PIN1_NODE DT_NODELABEL(power_pin1)
// #define POWER_PIN2_NODE DT_NODELABEL(power_pin2) 
// #define POWER_PIN3_NODE DT_NODELABEL(power_pin3)
// #define POWER_PIN4_NODE DT_NODELABEL(power_pin4)

// 检查节点是否启用
// #if !DT_NODE_HAS_STATUS(POWER_PIN1_NODE, okay)
// #error "power pin1 is disabled"
// #endif
// 获取 GPIO 规范
// static const struct gpio_dt_spec power_pin1 = GPIO_DT_SPEC_GET(POWER_PIN1_NODE, gpios);
// static const struct gpio_dt_spec power_pin2 = GPIO_DT_SPEC_GET(POWER_PIN2_NODE, gpios);
// static const struct gpio_dt_spec power_pin3 = GPIO_DT_SPEC_GET(POWER_PIN3_NODE, gpios);
// static const struct gpio_dt_spec power_pin4 = GPIO_DT_SPEC_GET(POWER_PIN4_NODE, gpios);
// #define ADC_NODE DT_NODELABEL(adc1)
// #define DAC_NODE DT_NODELABEL(dac1)
// #define PWM1_NODE DT_NODELABEL(pwm1)
const struct device *can_dev = DEVICE_DT_GET(DT_INST(0 ,vnd_canbus));
const struct device *motor1  = DEVICE_DT_GET(DT_NODELABEL(motor0));
// const struct device *adc_dev =DEVICE_DT_GET(ADC_NODE);
// struct adc_channel_cfg channel_cfg = {
//     .gain = ADC_GAIN_1,
//     .reference = ADC_REF_INTERNAL,
//     .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//     .channel_id = 8,  // 使用设备树中的通道8
//     // .resolution = 12, // 12位分辨率
// };
// uint16_t sample_buffer[1];
// const struct device *dac_dev = DEVICE_DT_GET(DAC_NODE);
// const struct device *led0 = DEVICE_DT_GET(DT_NODELABEL(led0));
// K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
// const struct device *steer_chassis = DEVICE_DT_GET(DT_INST(0, steer_chassis));
// const struct device *pwm_dev = DEVICE_DT_GET(PWM1_NODE);;
// int init_power_pins(void)
// {
//     int ret;

//     // 配置为输出
//     ret = gpio_pin_configure_dt(&power_pin1, GPIO_OUTPUT);
//     if (ret < 0) {
//         return ret;
//     }
    
//     ret = gpio_pin_configure_dt(&power_pin2, GPIO_OUTPUT);
//     if (ret < 0) {
//         return ret;
//     }
    
//     // ... 配置pin3和pin4
    
//     return 0;
// }

// // 控制引脚
// void control_power(void)
// {
//     // 设置引脚为高电平
//     gpio_pin_set_dt(&power_pin1, 1);
    
//     // 设置引脚为低电平
//     gpio_pin_set_dt(&power_pin2, 0);
    
//     // 读取引脚状态
//     int pin_state = gpio_pin_get_dt(&power_pin3);
// }
int main(void)
{
    // init_power_pins();
    // control_power();
    // LOG_INF("1");
    // printk("1");
 
    
    // k_thread_create(&feedback_thread, feedback_stack_area, K_THREAD_STACK_SIZEOF(feedback_stack_area),
    //                 feedback_thread_entry, NULL, NULL, NULL, 0, 0, K_NO_WAIT);
    // k_sleep(K_MSEC(50));
    
    // motor_set_mode(motor1, VO);
    motor_control(motor1, ENABLE_MOTOR);
    motor_set_speed(motor1, 20);
//   struct adc_sequence sequence = {
//         .channels = BIT(8), // 选择通道8
//         .buffer = sample_buffer,
//         .buffer_size = sizeof(sample_buffer),
//         .resolution = 12,
//     };
//     if (!adc_dev) {
//         printk("ADC device not found\n");
        
//     }
//     if (!dac_dev) {
//         LOG_ERR("Failed to get DAC device binding");
        
    // }
    // int ret;

    /* 获取 PWM 设备 */
    
    // if (!device_is_ready(pwm_dev)) {
    //     printk("PWM device not ready\n");
        
    // }

    /* 固定参数 PWM 输出 */
//     uint32_t period = 1000000;    // 1ms = 1kHz
//     uint32_t pulse = 500000;      // 50% 占空比
// ret = pwm_set(pwm_dev, 1, period, pulse, 0);
// 需要生成的 DAC 输出值（0-1023，假设是12位分辨率）
//     uint32_t dac_value = 512; // 设置为中间值，约等于 1.65V（假设参考电压为3.3V）
// int retd;
//     // 配置 DAC 输出通道和电压
//     retd = dac_write_value(dac_dev, 1, dac_value);  // 0为DAC通道编号，dac_value为要输出的数字值
//     if (retd) {
//         LOG_ERR("DAC write failed with error %d", retd);
        
//     }

//     LOG_INF("DAC value written: %d", dac_value);
    
//     // 持续输出变化值，模拟信号波形
//     for (int i = 0; i < 10; i++) {
//         dac_value = (i * 32) / 10; // 每次增大DAC值，产生不同电压
//         retd = dac_write_value(dac_dev, 1, dac_value);
//          int ret = adc_read(adc_dev, &sequence);
//     if (ret) {
//         printk("ADC read failed with error %d\n", ret);
//     } else {
//         printk("ADC value: %d\n", sample_buffer[0]);
//     }
//         if (retd) {
//             LOG_ERR("DAC write failed with error %d", ret);
            
//         }

//         LOG_INF("DAC value written: %d", dac_value);
//         k_msleep(500); // 每隔500ms写入一次新值
//     }

//     LOG_INF("DAC test complete.");
    // while (1) {
       
    //     // if (ret) {
    //     //     printk("Failed to set PWM: %d\n", ret);
    //     // }
    //     // printk("ar");
    //     // k_msleep(50);
    // }
   
   

    while (1) {
        float 
        motor1_rpm = motor_get_speed(motor1);
        LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm, (double)motor_get_speed(motor1));
        k_msleep(5);
        // k_sleep(K_FOREVER);
    }
}