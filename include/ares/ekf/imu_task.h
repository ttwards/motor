#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include "QuaternionEKF.h"
#include "algorithm.h"

#define X 0
#define Y 1
#define Z 2

#define FREQ 800

const int n = 800 / FREQ;

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
} INS_t;

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
static INS_t INS;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static const float gravity[3] = {0, 0, 9.81f};