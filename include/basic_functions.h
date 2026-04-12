#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include<vex.h>
#include<vector>
#include<parameters.h>

void init();
/**
* @brief 设置电机速度
* @param base 电机对象
* @param speed 速度(百分比单位 pct :-100~100)
*/
void set_motorpower(vex::motor motor,double speed,vex::brakeType type=vex::coast);

void set_motorpower(vex::motor_group motors,double speed,vex::brakeType type=vex::coast);

double function(double input);

double IMUHeading();

void resetHeading();

/**
  * @brief 弧度转角度转换函数
  * @param rad 以弧度为单位的角度值
  * @return 对应的角度值（单位：度），计算公式：弧度 * 180 / π
  * @example rad2deg(M_PI) 返回180.0
  */
 double rad_to_deg(double rad) ;

 /**
  * @brief 角度转弧度转换函数
  * @param deg 以度数为单位的角度值
  * @return 对应的弧度值，计算公式：度数 * π / 180
  * @example deg2rad(180.0) 返回M_PI
  */
 double deg_to_rad(double deg) ;

 double clamp(double value, double min, double max);

#endif
