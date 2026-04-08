#ifndef PARAMETERS_H_
#define PARAMETERS_H_

//线程更新时间 ms
const int REFRESH_TIME = 10;

//死区范围
const int DEAD_ZONE = 10;

// Inertial sensor correction coefficient, obtained by rotate robot 3600 deg and read
// the origin data got from `Inertial.rotation`.
// 惯性传感器校正系数，通过将机器人旋转3600度并读取
// 从“惯性旋转”中获得的原始数据。
const double IMU_MOD_COEFFICIENT = 3599.9;

// The distance traveled by the wheel when its motor rotates 1 radian
//const double WHEEL_TRANSITION_COEFFICIENT = 5.27;  // cm/rad
// 当车轮的电机旋转1弧度时，车轮行驶的距离 cm/rad
// (1/2*pi)*1*(pi*101.6)==5.08
const double WHEEL_TRANSITION_COEFFICIENT=40;

#endif