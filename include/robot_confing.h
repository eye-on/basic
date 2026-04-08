#ifndef ROBOT_CONFING_H_
#define ROBOT_CONFING_H_


#include "vex.h"

const bool is_BLUE =0; 

//惯性传感器对象
extern vex::inertial Inertial;

//遥控器对象
extern vex::brain Brain;

//视觉传感器
extern vex::motor sensor;

//主机对象
extern vex::controller Controller;

//Omni_motor 全向底盘电机
//fwdleft 左上电机
extern vex::motor motor_fl1;
extern vex::motor motor_fl2;
//fwdright 右上电机
extern vex::motor motor_fr1;
extern vex::motor motor_fr2;
//bwdleft 左下电机
extern vex::motor motor_bl1;
extern vex::motor motor_bl2;
//bwdright 右下电机
extern vex::motor motor_br1;
extern vex::motor motor_br2;

extern vex::motor middle_motor1;

extern vex::motor under_motor1;

extern vex::motor trans_motor1;
extern vex::motor trans_motor2;
extern vex::motor trans_motor3;

extern vex::motor under_overhang_motor;
extern vex::motor middle_overhang_motor;
extern vex::motor up_overhang_motor;

extern vex::motor up_motor1;

#endif
