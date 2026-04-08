#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "vex.h"
#include <vector>
#include "parameters.h"

/**
* @brief 设置电机速度
* @param base 电机对象
* @param speed 速度(百分比单位 pct :-100~100)
*/
void set_motorpower(vex::motor base,double speed,vex::brakeType type=vex::coast);

double function(double input);

//void enable_motor_analog(vex::motor,bool,int=100);

void enable_motor_analog(vex::motor,int,int=DEAD_ZONE,int=100);

//void enable_motor_discrete(vex::motor,bool,int=100);

void enable_motor_discrete(vex::motor,bool&,int=100);

void enable_motor_discrete(std::vector<vex::motor>,bool&,std::vector<int>);

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

#endif
