#ifndef PARAMETERS_H_
#define PARAMETERS_H_

//线程更新时间 ms
const int REFRESH_TIME_ms = 10;

//死区范围
const int DEAD_ZONE = 10;

// Inertial sensor correction coefficient, obtained by rotate robot 3600 deg and read
// the origin data got from `Inertial.rotation`.
// 惯性传感器校正系数，通过将机器人旋转3600度并读取
// 从“惯性旋转”中获得的原始数据。
const double IMU_MOD_COEFFICIENT = 3599.9;

//const double M_PI=3.1416;

const double Wheel_Diameter_cm=9;

const double Wheel_Radius_cm=4.5;

const double Turn_Radius_cm=15;

#endif