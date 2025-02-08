#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include "QuaternionEKF.h"
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

static const int X = 0;
static const int Y = 1;
static const int Z = 2;

#define FREQ 800

const int n = 800 / FREQ;

typedef void (*update_cb_t)(QEKF_INS_t *QEKF);

typedef struct {
	// IMU量测值
	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度

	float lpf_Accel[3]; // 加速度低通滤波

	uint8_t flag;

	int gyro_prev_cyc;
	int accel_prev_cyc;

	float AccelLPF;

	int gyro_curr_cyc;
	int accel_curr_cyc;

	update_cb_t update_cb;

	const struct device *accel_dev;
	const struct device *gyro_dev;
} INS_t;

typedef struct {
	bool flag;
	float Gyro[3]; // 角速度
} IMU_Bias_t;

/* 用于修正安装误差的参数 */
typedef struct {
	uint8_t flag;

	float scale[3];

	float Yaw;
	float Pitch;
	float Roll;
} IMU_Param_t;

static struct sensor_trigger accel_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};
static struct sensor_trigger gyro_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_GYRO_XYZ,
};

static IMU_Param_t IMU_Param;

#ifdef DT_HAS_CHOSEN(zephyr_ccm)
static __ccm_bss_section INS_t INS;
#else
static INS_t INS;
#endif

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static const float gravity[3] = {0, 0, 9.81f};

#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

#ifdef CONFIG_IMU_PWM_TEMP_CTRL
static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_CHOSEN(ares_pwm));
static float target_temp = 50.0f;
static float current_temp = 25.0f;
static float temp_pwm_output = 0.0f;
#ifdef DT_NODE_EXISTS(DT_NODELABEL(imu_temp_pid))
PID_NEW_INSTANCE(DT_NODELABEL(imu_temp_pid), ins)
struct pid_data *temp_pwm_pid = &PID_INS_NAME(DT_NODELABEL(imu_temp_pid), ins);
#else
#error "No PID instance for IMU temperature control"
#endif // DT_NODELABEL(imu_temp_pid)
#endif // CONFIG_IMU_PWM_TEMP_CTRL

#if DT_HAS_CHOSEN(ares_bias)
#define GYRO_BIAS_NODE DT_CHOSEN(ares_bias)
#if DT_NODE_HAS_PROP(GYRO_BIAS_NODE, gyro_bias)
#endif // DT_NODE_HAS_PROP(GYRO_BIAS_NODE, gyro_bias)
#endif // DT_HAS_CHOSEN(ares_bias)