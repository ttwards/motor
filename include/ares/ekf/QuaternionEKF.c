/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 *
 ******************************************************************************
 */
#include "QuaternionEKF.h"
#include "kalman_filter.h"
#include <math.h>
#include "kalman_filter.c"
#include <zephyr/kernel.h>
#include "algorithm.h"
#include <string.h>

#if DT_HAS_CHOSEN(zephyr_ccm)
__ccm_bss_section QEKF_INS_t QEKF_INS;
__ccm_data_section float IMU_QuaternionEKF_P[16] = default_EKF_P;
__ccm_bss_section float IMU_QuaternionEKF_K[12];
__ccm_bss_section float IMU_QuaternionEKF_H[12];
#else
QEKF_INS_t QEKF_INS;
float IMU_QuaternionEKF_P[16] = default_EKF_P;
float IMU_QuaternionEKF_K[12];
float IMU_QuaternionEKF_H[12];
#endif

const float IMU_QuaternionEKF_F[16] = default_EKF_F;

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

void CalcBias(float *q, float *accel, float g, float *bias)
{
	// 理想情况下，静止时加速度计受力只有重力
	// 将重力从地面坐标系(0,0,-g)旋转到设备坐标系得到理想值
	float R[3][3];
	R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	R[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
	R[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);

	R[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
	R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
	R[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);

	R[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
	R[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
	R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

	// 计算理想的加速度计读数
	float ideal_accel[3];
	ideal_accel[0] = g * R[2][0]; // 重力在设备x轴的分量
	ideal_accel[1] = g * R[2][1]; // 重力在设备y轴的分量
	ideal_accel[2] = g * R[2][2]; // 重力在设备z轴的分量

	// 零飘 = 实际读数 - 理想值
	bias[0] = accel[0] - ideal_accel[0];
	bias[1] = accel[1] - ideal_accel[1];
	bias[2] = accel[2] - ideal_accel[2];
}

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1, float process_noise2,
			    float measure_noise, float lambda, float lpf)
{
	QEKF_INS.Initialized = 1;
	QEKF_INS.Q1 = process_noise1;
	QEKF_INS.Q2 = process_noise2;
	QEKF_INS.R = measure_noise;
	QEKF_INS.ChiSquareTestThreshold = 1e-8;
	QEKF_INS.ConvergeFlag = 0;
	QEKF_INS.ErrorCount = 0;
	if (lambda > 1) {
		lambda = 1;
	}
	QEKF_INS.lambda = lambda;
	QEKF_INS.accLPFcoef = lpf;

	// 初始化矩阵维度信息
	Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 4, 0, 3);
	Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

	// 姿态初始化
	for (int i = 0; i < 4; i++) {
		QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i];
	}

	// 自定义函数初始化,用于扩展或增加kf的基础功能
	QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
	// We don't probe bias now.
	QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
	QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

	// 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
	QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
	QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
	memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az,
			      float accel_dt, float gyro_dt)
{
	// 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
	static float halfgxdt, halfgydt, halfgzdt;
	static float accelInvNorm;

	/*   F, number with * represent vals to be set
	 0      1*     2*     3*
	 4*     5      6*     7*
	 8*     9*    10     11*
	12*    13*    14*    15
	*/
	QEKF_INS.dt = gyro_dt;

	QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
	QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
	QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

	// set F
	halfgxdt = 0.5f * QEKF_INS.Gyro[0] * gyro_dt;
	halfgydt = 0.5f * QEKF_INS.Gyro[1] * gyro_dt;
	halfgzdt = 0.5f * QEKF_INS.Gyro[2] * gyro_dt;

	// 此部分设定状态转移矩阵F的左上角部分
	// 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
	// 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

	QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[4] = halfgxdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[7] = -halfgydt;

	QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[11] = halfgxdt;

	QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgzdt;
	QEKF_INS.IMU_QuaternionEKF.F_data[13] = halfgydt;
	QEKF_INS.IMU_QuaternionEKF.F_data[14] = -halfgxdt;

	// accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
	if (QEKF_INS.UpdateCount == 200) // 如果是第一次进入,需要初始化低通滤波
	{
		QEKF_INS.Accel[0] = ax;
		QEKF_INS.Accel[1] = ay;
		QEKF_INS.Accel[2] = az;
	}
	QEKF_INS.Accel[0] =
		QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (accel_dt + QEKF_INS.accLPFcoef) +
		(ax - QEKF_INS.AccelBias[0]) * accel_dt / (accel_dt + QEKF_INS.accLPFcoef);
	QEKF_INS.Accel[1] =
		QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (accel_dt + QEKF_INS.accLPFcoef) +
		(ay - QEKF_INS.AccelBias[1]) * accel_dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
	QEKF_INS.Accel[2] =
		QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (accel_dt + QEKF_INS.accLPFcoef) +
		(az - QEKF_INS.AccelBias[2]) * accel_dt / (accel_dt + QEKF_INS.accLPFcoef);

	// set z,单位化重力加速度向量
	accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
			       QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
			       QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
	for (uint8_t i = 0; i < 3; ++i) {
		QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] =
			QEKF_INS.Accel[i] * accelInvNorm; // 用加速度向量更新量测值
	}

	// get body state
	QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
					    QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
					    QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
	QEKF_INS.accl_norm = 1.0f / accelInvNorm;

	// 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
	// 稍后在最后的姿态更新部分会利用StableFlag来确定
	float acc[3] = {ax, ay, az};
	if (QEKF_INS.gyro_norm < 0.002f && fabsf(NormOf3d(acc) - 9.8f) < 0.25f) {
		QEKF_INS.StableFlag = 1;
	} else {
		QEKF_INS.StableFlag = 0;
	}
	// set Q R,过程噪声和观测噪声矩阵
	QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
	QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
	QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
	QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

#if DT_HAS_CHOSEN(ares_bias) && CONFIG_AUTO_PROBE_GYRO_BIAS
#error Do not use bias specified in dts and auto probe at the same time!!
#endif // DT_HAS_CHOSEN(ares_bias) && CONFIG_AUTO_PROBE_BIAS

#if CONFIG_AUTO_PROBE_GYRO_BIAS
	if (QEKF_INS.StableFlag) {
		// We update gyro bias with a low pass filter
		QEKF_INS.GyroBias[0] = QEKF_INS.GyroBias[0] * 0.9992f + gx * 0.0008f;
		QEKF_INS.GyroBias[1] = QEKF_INS.GyroBias[1] * 0.9992f + gy * 0.0008f;
		QEKF_INS.GyroBias[2] = QEKF_INS.GyroBias[2] * 0.9992f + gz * 0.0008f;
		float bias[3];
#ifndef PHY_G
		QEKF_INS.g = QEKF_INS.g * 0.992f + NormOf3d(acc) * 0.008f;
#else
		QEKF_INS.g = PHY_G;
#endif // PHY_G
		CalcBias(QEKF_INS.q, acc, QEKF_INS.g, bias);
		QEKF_INS.AccelBias[0] = QEKF_INS.AccelBias[0] * 0.992f + bias[0] * 0.008f;
		QEKF_INS.AccelBias[1] = QEKF_INS.AccelBias[1] * 0.992f + bias[1] * 0.008f;
		QEKF_INS.AccelBias[2] = QEKF_INS.AccelBias[2] * 0.992f + bias[2] * 0.008f;
	}
#endif // CONFIG_AUTO_PROBE_GYRO_BIAS

#if !DT_HAS_CHOSEN(ares_bias) && !CONFIG_AUTO_PROBE_GYRO_BIAS
	QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
	QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
	QEKF_INS.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移
#endif                            // DT_HAS_CHOSEN(ares_bias)

	// 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
	Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

	// 获取融合后的数据,包括四元数和xy零飘值
	QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
	QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
	QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
	QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

	// 利用四元数反解欧拉角
	QEKF_INS.Yaw =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Pitch =
		atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
		       2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) -
			       1.0f) *
		57.295779513f;
	QEKF_INS.Roll =
		asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) *
		57.295779513f;

	// get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
	if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
		QEKF_INS.YawRoundCount--;
	} else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
		QEKF_INS.YawRoundCount++;
	}
	QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
	QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
	QEKF_INS.UpdateCount++; // 初始化低通滤波用,计数测试用
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
	static float doubleq0, doubleq1, doubleq2, doubleq3;
	/* H
	0     1     2     3
	4     5     6     7
	8     9    10    11
	*/
	// set H
	doubleq0 = 2 * kf->xhatminus_data[0];
	doubleq1 = 2 * kf->xhatminus_data[1];
	doubleq2 = 2 * kf->xhatminus_data[2];
	doubleq3 = 2 * kf->xhatminus_data[3];

	memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

	kf->H_data[0] = -doubleq2;
	kf->H_data[1] = doubleq3;
	kf->H_data[2] = -doubleq0;
	kf->H_data[3] = doubleq1;

	kf->H_data[4] = doubleq1;
	kf->H_data[5] = doubleq0;
	kf->H_data[6] = doubleq3;
	kf->H_data[7] = doubleq2;

	kf->H_data[8] = doubleq0;
	kf->H_data[9] = -doubleq1;
	kf->H_data[10] = -doubleq2;
	kf->H_data[11] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
	static float q0, q1, q2, q3;

	kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
	kf->temp_matrix.numRows = kf->H.numRows;
	kf->temp_matrix.numCols = kf->Pminus.numCols;
	kf->MatStatus =
		Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
	kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
	kf->temp_matrix1.numCols = kf->HT.numCols;
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT,
					&kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
	kf->S.numRows = kf->R.numRows;
	kf->S.numCols = kf->R.numCols;
	kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
	kf->MatStatus =
		Matrix_Inverse(&kf->S, &kf->temp_matrix1); // temp_matrix1 = inv(H·P'(k)·HT + R)

	q0 = kf->xhatminus_data[0];
	q1 = kf->xhatminus_data[1];
	q2 = kf->xhatminus_data[2];
	q3 = kf->xhatminus_data[3];

	kf->temp_vector.numRows = kf->H.numRows;
	kf->temp_vector.numCols = 1;
	// 计算预测得到的重力加速度方向(通过姿态获取的)
	kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
	kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
	kf->temp_vector_data[2] =
		q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

	// 计算预测值和各个轴的方向余弦
	for (uint8_t i = 0; i < 3; ++i) {
		QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
	}

	// 利用加速度计数据修正
	kf->temp_vector1.numRows = kf->z.numRows;
	kf->temp_vector1.numCols = 1;
	kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector,
					&kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

	// chi-square test,卡方检验
	kf->temp_matrix.numRows = kf->temp_vector1.numRows;
	kf->temp_matrix.numCols = 1;
	kf->MatStatus = Matrix_Multiply(
		&kf->temp_matrix1, &kf->temp_vector1,
		&kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
	kf->temp_vector.numRows = 1;
	kf->temp_vector.numCols = kf->temp_vector1.numRows;
	kf->MatStatus = Matrix_Transpose(&kf->temp_vector1,
					 &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
	kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
	// rk is small,filter converged/converging
	if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
		QEKF_INS.ConvergeFlag = 1;
	}
	// rk is bigger than three but once converged
	if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
		if (QEKF_INS.StableFlag) {
			QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
		} else {
			QEKF_INS.ErrorCount = 0;
		}

		if (QEKF_INS.ErrorCount > 50) {
			// 滤波器发散
			QEKF_INS.ConvergeFlag = 0;
			kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
		} else {
			//  残差未通过卡方检验 仅预测
			//  xhat(k) = xhat'(k)
			//  P(k) = P'(k)
			memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
			memcpy(kf->P_data, kf->Pminus_data,
			       sizeof_float * kf->xhatSize * kf->xhatSize);
			kf->SkipEq5 = TRUE; // part5 is P updating
			return;
		}
	} else // if divergent or rk is not that big/acceptable,use adaptive gain
	{
		// scale adaptive,rk越小则增益越大,否则更相信预测值
		if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold &&
		    QEKF_INS.ConvergeFlag) {
			QEKF_INS.AdaptiveGainScale =
				(QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) /
				(0.9f * QEKF_INS.ChiSquareTestThreshold);
		} else {
			QEKF_INS.AdaptiveGainScale = 1;
		}
		QEKF_INS.ErrorCount = 0;
		kf->SkipEq5 = FALSE;
	}

	// cal kf-gain K
	kf->temp_matrix.numRows = kf->Pminus.numRows;
	kf->temp_matrix.numCols = kf->HT.numCols;
	kf->MatStatus =
		Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

	// implement adaptive
	for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i) {
		kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
	}

	kf->temp_vector.numRows = kf->K.numRows;
	kf->temp_vector.numCols = 1;
	kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
					&kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

	kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
	memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
	memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
	memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
