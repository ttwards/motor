#include <zephyr/drivers/sensor.h>
#include "ares/ekf/QuaternionEKF.h"
#include "imu_task.h"
#include "zephyr/devicetree.h"
#include "zephyr/drivers/pid.h"
#include "zephyr/kernel.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "zephyr/sys/printk.h"
#include "zephyr/sys/time_units.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/_types.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "matrix_storage.h"
#include "algorithm.h"

LOG_MODULE_REGISTER(imu_task, LOG_LEVEL_INF);

static int count = 0;

static IMU_Param_t IMU_Param;

#if DT_HAS_CHOSEN(zephyr_ccm)
__ccm_bss_section INS_t INS;
#elif DT_HAS_CHOSEN(zephyr_dtcm)
__dtcm_bss_section INS_t INS;
#else
INS_t INS;
#endif

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static float current_temp = 25.0f;

#ifdef CONFIG_IMU_PWM_TEMP_CTRL
static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_CHOSEN(ares_pwm));
static float target_temp = 50.0f;
static float temp_pwm_output = 19900000.0f;
#ifdef DT_NODE_EXISTS(DT_NODELABEL(imu_temp_pid))
PID_NEW_INSTANCE(DT_NODELABEL(imu_temp_pid), ins)
struct pid_data *temp_pwm_pid = &PID_INS_NAME(DT_NODELABEL(imu_temp_pid), ins);
#else
#error "No PID instance for IMU temperature control"
#endif // DT_NODELABEL(imu_temp_pid)
#endif // CONFIG_IMU_PWM_TEMP_CTRL

float IMU_temp_read(const struct device *dev)
{
	struct sensor_value temp;
	sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
	current_temp = sensor_value_to_double(&temp);
	return current_temp;
}

int IMU_temp_pwm_set(const struct device *dev)
{
#ifdef CONFIG_IMU_PWM_TEMP_CTRL
	pid_calc(temp_pwm_pid);
	return pwm_set_pulse_dt(&pwm, ((int)temp_pwm_output < 0) ? 0 : (int)temp_pwm_output);
#else
	return 0;
#endif // CONFIG_IMU_PWM_TEMP_CTRL
}

static inline void IMU_Sensor_handle_update(INS_t *data)
{
	if (data->flag != 0b11) {
		return;
	}

	data->flag = 0;
	count++;

	float accel_dt =
		k_cyc_to_us_near32(data->accel_curr_cyc - data->accel_prev_cyc) * 0.000001f;
	float gyro_dt = k_cyc_to_us_near32(data->gyro_curr_cyc - data->gyro_prev_cyc) * 0.000001f;

	IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y],
				 INS.Accel[Z], accel_dt, gyro_dt);

#ifdef CONFIG_IMU_PWM_TEMP_CTRL
	if (count % 50 == 0) {
		IMU_temp_read(data->accel_dev);

		IMU_temp_pwm_set(data->accel_dev);

		if (current_temp >= 65) {
			printk("Current Temp: %.2f, PWM: %d\n", (double)current_temp,
			       (int)temp_pwm_output);
		}
	}
#endif // CONFIG_IMU_PWM_TEMP_CTRL

	data->gyro_prev_cyc = data->gyro_curr_cyc;
	data->accel_prev_cyc = data->accel_curr_cyc;

	if (data->update_cb != NULL) {
		data->update_cb(&QEKF_INS);
	}
}

void IMU_Sensor_set_update_cb(update_cb_t cb)
{
	INS.update_cb = cb;
}

#ifdef CONFIG_IMU_PWM_TEMP_CTRL
void IMU_Sensor_set_IMU_temp(float temp)
{
	target_temp = temp;
}
#endif // CONFIG_IMU_PWM_TEMP_CTRL

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(const struct device *accel_dev, const struct device *gyro_dev,
			   float *init_q4, float *accel)
{
	float acc_init[3] = {0};
	float single_acc_bias[3] = {0};
	float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
	float axis_rot[3] = {0};           // 旋转轴
	float acc[3] = {0};
	float gyro[3] = {0};
	// 读取100次加速度计数据,取平均值作为初始值
	struct sensor_value accel_data[3];
	struct sensor_value gyro_data[3];
	QEKF_INS.UpdateCount = 0;

	INS.accel_curr_cyc = k_cycle_get_32();

#ifdef CONFIG_IMU_PWM_TEMP_CTRL
	INS.accel_prev_cyc = INS.accel_curr_cyc;
	INS.accel_curr_cyc = k_cycle_get_32();
	sensor_sample_fetch(accel_dev);
	IMU_temp_read(accel_dev);
	IMU_temp_pwm_set(accel_dev);
#endif // CONFIG_IMU_PWM_TEMP_CTRL

	int sample_cnt = 0;
	for (int i = 0; i < 600; ++i) {
		sensor_sample_fetch(accel_dev);
		sensor_sample_fetch(gyro_dev);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
		sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
		acc[X] = sensor_value_to_float(&accel_data[X]);
		acc[Y] = sensor_value_to_float(&accel_data[Y]);
		acc[Z] = sensor_value_to_float(&accel_data[Z]);

		float g = sqrtf(acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z]);
		if (fabsf(g - PHY_G) > 0.16f) {
			continue;
		}
		acc_init[X] += acc[X];
		acc_init[Y] += acc[Y];
		acc_init[Z] += acc[Z];
		gyro[X] += sensor_value_to_float(&gyro_data[X]);
		gyro[Y] += sensor_value_to_float(&gyro_data[Y]);
		gyro[Z] += sensor_value_to_float(&gyro_data[Z]);
		sample_cnt++;

		k_msleep(1);
	}

#ifndef PHY_G
	QEKF_INS.g = sqrtf(acc_init[X] * acc_init[X] + acc_init[Y] * acc_init[Y] +
			   acc_init[Z] * acc_init[Z]) /
		     QEKF_INS.UpdateCount;
#else
	QEKF_INS.g = PHY_G;
#endif // PHY_G

	if (matrix_storage_exists()) {
		float matrix1[MATRIX_ROWS][MATRIX_COLS];
		float matrix2[MATRIX_ROWS][MATRIX_COLS];
		matrix_storage_read(matrix1, matrix2);
		QEKF_INS.AccelBias[X] = matrix1[0][0];
		QEKF_INS.AccelBias[Y] = matrix1[0][1];
		QEKF_INS.AccelBias[Z] = matrix1[0][2];
		QEKF_INS.AccelBeta[X] = matrix1[1][0];
		QEKF_INS.AccelBeta[Y] = matrix1[1][1];
		QEKF_INS.AccelBeta[Z] = matrix1[1][2];
		QEKF_INS.GyroBias[X] = matrix1[2][0];
		QEKF_INS.GyroBias[Y] = matrix1[2][1];
		QEKF_INS.GyroBias[Z] = matrix1[2][2];
		printk("AccelBias: %f, %f, %f\n", QEKF_INS.AccelBias[X], QEKF_INS.AccelBias[Y],
		       QEKF_INS.AccelBias[Z]);
		printk("AccelBeta: %f, %f, %f\n", QEKF_INS.AccelBeta[X], QEKF_INS.AccelBeta[Y],
		       QEKF_INS.AccelBeta[Z]);
		printk("GyroBias: %f, %f, %f\n", QEKF_INS.GyroBias[X], QEKF_INS.GyroBias[Y],
		       QEKF_INS.GyroBias[Z]);
		QEKF_INS.hasStoredBias = true;
	} else {
		LOG_ERR("No stored bias, using default values");
		LOG_ERR("Please calibrate the sensor");
		QEKF_INS.AccelBeta[X] = 1;
		QEKF_INS.AccelBeta[Y] = 1;
		QEKF_INS.AccelBeta[Z] = 1;
		QEKF_INS.AccelBias[X] = 0;
		QEKF_INS.AccelBias[Y] = 0;
		QEKF_INS.AccelBias[Z] = 0;
		QEKF_INS.GyroBias[X] = 0;
		QEKF_INS.GyroBias[Y] = 0;
		QEKF_INS.GyroBias[Z] = 0;
	}

	if (sample_cnt == 0) {
		acc_init[X] = 0;
		acc_init[Y] = 0;
		acc_init[Z] = PHY_G;
	} else {
		for (uint8_t i = 0; i < 3; ++i) {
			acc_init[i] /= sample_cnt;
			acc_init[i] -= QEKF_INS.AccelBias[i];
			acc_init[i] /= QEKF_INS.AccelBeta[i];
			if (QEKF_INS.GyroBias[i] == 0) {
				QEKF_INS.GyroBias[i] = gyro[i] / sample_cnt;
			}
		}
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
	printk("Init Quaternion: %f, %f, %f, %f\n", init_q4[0], init_q4[1], init_q4[2], init_q4[3]);
	printk("Accel: %f, %f, %f\n", acc_init[X], acc_init[Y], acc_init[Z]);
}

int acnt = 0;
static void IMU_Sensor_trig_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
	if (trigger->type != SENSOR_TRIG_DATA_READY) {
		return;
	}
	uint32_t current_cyc = k_cycle_get_32();
	sensor_sample_fetch(dev);
	acnt++;

	if (trigger->chan == SENSOR_CHAN_ACCEL_XYZ) {
		struct sensor_value accel_data[3];
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);

		INS.flag |= 1;
		INS.accel_curr_cyc = current_cyc;

		INS.Accel[X] = (sensor_value_to_float(&accel_data[X]) - QEKF_INS.AccelBias[X]) /
			       QEKF_INS.AccelBeta[X];
		INS.Accel[Y] = (sensor_value_to_float(&accel_data[Y]) - QEKF_INS.AccelBias[Y]) /
			       QEKF_INS.AccelBeta[Y];
		INS.Accel[Z] = (sensor_value_to_float(&accel_data[Z]) - QEKF_INS.AccelBias[Z]) /
			       QEKF_INS.AccelBeta[Z];
	} else if (trigger->chan == SENSOR_CHAN_GYRO_XYZ) {
		struct sensor_value gyro_data[3];
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);

		INS.flag |= 2;
		INS.gyro_curr_cyc = current_cyc;

		INS.Gyro[0] = sensor_value_to_float(&gyro_data[0]) - QEKF_INS.GyroBias[0];
		INS.Gyro[1] = sensor_value_to_float(&gyro_data[1]) - QEKF_INS.GyroBias[1];
		INS.Gyro[2] = sensor_value_to_float(&gyro_data[2]) - QEKF_INS.GyroBias[2];
	}

	IMU_Sensor_handle_update(&INS);
}

float init_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // [w, x, y, z]

static struct sensor_trigger accel_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};
static struct sensor_trigger gyro_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_GYRO_XYZ,
};

void IMU_Sensor_trig_init(const struct device *accel_dev, const struct device *gyro_dev)
{
	int ret = 0;
#ifdef CONFIG_IMU_PWM_TEMP_CTRL
	pwm_set_pulse_dt(&pwm, 20000000);

	pid_reg_input(temp_pwm_pid, &current_temp, &target_temp);
	pid_reg_output(temp_pwm_pid, &temp_pwm_output);
	pid_reg_time(temp_pwm_pid, &INS.accel_curr_cyc, &INS.accel_prev_cyc);

	if (!pwm_is_ready_dt(&pwm)) {
		printk("Error: PWM device %s is not ready\n", pwm.dev->name);
		return;
	}
#endif // CONFIG_IMU_PWM_TEMP_CTRL

	IMU_Param.scale[X] = 1;
	IMU_Param.scale[Y] = 1;
	IMU_Param.scale[Z] = 1;
	IMU_Param.Yaw = 0;
	IMU_Param.Pitch = 0;
	IMU_Param.Roll = 0;
	IMU_Param.flag = 1;

	int current_cyc = k_cycle_get_32();
	INS.gyro_prev_cyc = current_cyc;
	INS.accel_prev_cyc = current_cyc;
	INS.flag = 0;
	INS.accel_dev = accel_dev;
	INS.gyro_dev = gyro_dev;

	float init_quaternion[4] = {0};
	InitQuaternion(accel_dev, gyro_dev, init_quaternion, INS.lpf_Accel);

	// 调用初始化
	IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 100000, 0.9999996, 0);

	sensor_trigger_set(accel_dev, &accel_trig, IMU_Sensor_trig_handler);
	sensor_trigger_set(gyro_dev, &gyro_trig, IMU_Sensor_trig_handler);
}