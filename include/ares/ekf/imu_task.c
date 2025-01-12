#include <zephyr/drivers/sensor.h>
#include "QuaternionEKF.c"
#include "imu_task.h"
#include "zephyr/sys/time_units.h"
#include "algorithm.c"

static int count = 0;

static inline void IMU_Sensor_handle_update(INS_t *data)
{
	// BMI088 has frequency at about 1800Hz
	// We update at 300Hz
	if (data->flag < 3) {
		return;
	}

	data->flag = 0;
	count++;

	float accel_dt =
		k_cyc_to_us_near32(data->accel_curr_cyc - data->accel_prev_cyc) * 0.000001f;
	float gyro_dt = k_cyc_to_us_near32(data->gyro_curr_cyc - data->gyro_prev_cyc) * 0.000001f;

	IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y],
				 INS.Accel[Z], accel_dt, gyro_dt);

	data->gyro_prev_cyc = data->gyro_curr_cyc;
	data->accel_prev_cyc = data->accel_curr_cyc;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(const struct device *dev, float *init_q4, float *accel)
{
	float acc_init[3] = {0};
	float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
	float axis_rot[3] = {0};           // 旋转轴
					   // 读取100次加速度计数据,取平均值作为初始值
	struct sensor_value accel_data[3];

	for (uint8_t i = 0; i < 100; ++i) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
		acc_init[X] += sensor_value_to_float(&accel_data[X]);
		acc_init[Y] += sensor_value_to_float(&accel_data[Y]);
		acc_init[Z] += sensor_value_to_float(&accel_data[Z]);
		k_msleep(1);
	}
	for (uint8_t i = 0; i < 3; ++i) {
		acc_init[i] /= 100;
	}
	accel[X] = acc_init[X];
	accel[Y] = acc_init[Y];
	accel[Z] = acc_init[Z];

	Norm3d(acc_init);
	// 计算原始加速度矢量和导航系重力加速度矢量的夹角
	float angle = acosf(Dot3d(acc_init, gravity_norm));
	Cross3d(acc_init, gravity_norm, axis_rot);
	Norm3d(axis_rot);
	init_q4[0] = cosf(angle / 2.0f);
	for (uint8_t i = 0; i < 2; ++i) {
		init_q4[i + 1] =
			axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
	}
}

static void IMU_Sensor_trig_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
	if (trigger->type != SENSOR_TRIG_DATA_READY) {
		return;
	}
	int current_cyc = k_cycle_get_32();
	sensor_sample_fetch(dev);
	if (trigger->chan == SENSOR_CHAN_ACCEL_XYZ) {
		struct sensor_value accel_data[3];
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);

		INS.flag |= 1;
		INS.accel_curr_cyc = current_cyc;

		INS.Accel[X] = sensor_value_to_float(&accel_data[X]);
		INS.Accel[Y] = sensor_value_to_float(&accel_data[Y]);
		INS.Accel[Z] = sensor_value_to_float(&accel_data[Z]);
	} else if (trigger->chan == SENSOR_CHAN_GYRO_XYZ) {
		struct sensor_value gyro_data[3];
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);

		INS.flag |= 2;
		INS.gyro_curr_cyc = current_cyc;

		INS.Gyro[0] = sensor_value_to_float(&gyro_data[0]);
		INS.Gyro[1] = sensor_value_to_float(&gyro_data[1]);
		INS.Gyro[2] = sensor_value_to_float(&gyro_data[2]);
	}

	IMU_Sensor_handle_update(&INS);
}

float init_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // [w, x, y, z]

static void IMU_Sensor_trig_init(const struct device *accel_dev, const struct device *gyro_dev)
{
	IMU_Param.scale[X] = 1;
	IMU_Param.scale[Y] = 1;
	IMU_Param.scale[Z] = 1;
	IMU_Param.Yaw = 0;
	IMU_Param.Pitch = 0;
	IMU_Param.Roll = 0;
	IMU_Param.flag = 1;

	float init_quaternion[4] = {0};
	InitQuaternion(accel_dev, init_quaternion, INS.lpf_Accel);

	// 调用初始化
	IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 100000, 1, 0.95);

	int current_cyc = k_cycle_get_32();
	INS.gyro_prev_cyc = current_cyc;
	INS.accel_prev_cyc = current_cyc;
	INS.flag = 0;
	sensor_trigger_set(accel_dev, &accel_trig, IMU_Sensor_trig_handler);
	sensor_trigger_set(gyro_dev, &gyro_trig, IMU_Sensor_trig_handler);
}