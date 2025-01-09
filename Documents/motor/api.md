# 电机驱动API
```c
motor_driver_api {
    float motor_get_speed(const struct device *dev);
    float motor_get_torque(const struct device *dev);
    float motor_get_angle(const struct device *dev);
    int motor_set_speed(const struct device *dev, float speed_rpm);
    int motor_set_torque(const struct device *dev, float torque);
    int motor_set_angle(const struct device *dev, float angle);
    int motor_set_mode(const struct device *dev, enum motor_mode mode);
    void motor_control(const struct device *dev, enum motor_cmd cmd);
};
```
```c
/**
 * @brief 电机工作模式
 * 
 * MIT: MIT模式
 * PV: 位置-速度控制
 * VO: 速度控制  
 * MULTILOOP: 多环串联控制
 */
enum motor_mode { MIT, PV, VO, MULTILOOP };
```
```c
/**
 * @brief 电机控制命令
 */
enum motor_cmd { ENABLE_MOTOR, DISABLE_MOTOR, SET_ZERO_OFFSET, CLEAR_PID, CLEAR_ERROR };
```
## 电机状态获取
`motor_get_speed(const struct device *dev)`: 获取电机当前速度，返回值为电机当前速度，单位为rpm。
`motor_get_torque(const struct device *dev)`: 获取电机当前扭矩，返回值为电机当前扭矩，单位为N*m。
`motor_get_angle(const struct device *dev)`: 获取电机当前角度，返回值为电机当前角度，单位为°。
`motor_set_speed(const struct device *dev, float speed_rpm)`: 设置电机目标速度，参数为目标速度，单位为rpm。
`motor_set_torque(const struct device *dev, float torque)`: 设置电机目标扭矩，参数为目标扭矩，单位为N*m。
`motor_set_angle(const struct device *dev, float angle)`: 设置电机目标角度，参数为目标角度，单位为°。
`motor_set_mode(const struct device *dev, enum motor_mode mode)`: 设置电机工作模式，参数为工作模式。
`motor_control(const struct device *dev, enum motor_cmd cmd)`: 控制电机，参数为控制命令。