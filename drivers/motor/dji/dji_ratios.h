/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define M3508_CURRENT2TORQUE 0.00002200f
#define M3508_TORQUE2CURRENT 45000.0422411f
#define M3508_SPEED2RPM      1.0f
#define M3508_RPM2SPEED      1.0f
#define M3508_ANGLE2DEGREE   0.043945312f
#define M3508_DEGREE2ANGLE   22.75277778f

#define GM6020_CURRENT2TORQUE 0.00010986f
#define GM6020_TORQUE2CURRENT 9102.22222f
#define GM6020_SPEED2RPM      1.0f
#define GM6020_RPM2SPEED      1.0f
#define GM6020_ANGLE2DEGREE   0.043950678f
#define GM6020_DEGREE2ANGLE   22.75277778f

#define M2006_CURRENT2TORQUE 0.0000089606f
#define M2006_TORQUE2CURRENT 111600.0f
#define M2006_SPEED2RPM      1.0f
#define M2006_RPM2SPEED      1.0f
#define M2006_ANGLE2DEGREE   0.043950678f
#define M2006_DEGREE2ANGLE   22.75277778f

#define DM_MOTOR_SPEED2RPM    0.001f
#define DM_MOTOR_RPM2SPEED    1.0f
#define DM_MOTOR_ANGLE2DEGREE 0.001f

#define M3508_CONVERT_NUM    0
#define GM6020_CONVERT_NUM   1
#define M2006_CONVERT_NUM    2
#define DM_MOTOR_CONVERT_NUM 3

#define CURRENT2TORQUE 0
#define TORQUE2CURRENT 1
#define SPEED2RPM      2
#define RPM2SPEED      3
#define ANGLE2DEGREE   4
#define DEGREE2ANGLE   5

const float convert[4][6] = {
	{M3508_CURRENT2TORQUE, M3508_TORQUE2CURRENT, M3508_SPEED2RPM, M3508_RPM2SPEED,
	 M3508_ANGLE2DEGREE, M3508_DEGREE2ANGLE},

	{GM6020_CURRENT2TORQUE, GM6020_TORQUE2CURRENT, GM6020_SPEED2RPM, GM6020_RPM2SPEED,
	 GM6020_ANGLE2DEGREE, GM6020_DEGREE2ANGLE},

	{M2006_CURRENT2TORQUE, M2006_TORQUE2CURRENT, M2006_SPEED2RPM, M2006_RPM2SPEED,
	 M2006_ANGLE2DEGREE, M2006_DEGREE2ANGLE},

	{0.0f, 0.0f, DM_MOTOR_SPEED2RPM, DM_MOTOR_RPM2SPEED, DM_MOTOR_ANGLE2DEGREE, 0.0},
};
