#include "robot_confing.h"

vex::motor motor_fr1=vex::motor(vex::PORT17,vex::ratio6_1,true);
vex::motor motor_fr2=vex::motor(vex::PORT18,vex::ratio6_1,false);

vex::motor motor_br1=vex::motor(vex::PORT15,vex::ratio6_1,false);
vex::motor motor_br2=vex::motor(vex::PORT16,vex::ratio6_1,true);

vex::motor motor_fl1=vex::motor(vex::PORT13,vex::ratio6_1,false);
vex::motor motor_fl2=vex::motor(vex::PORT14,vex::ratio6_1,true);

vex::motor motor_bl1=vex::motor(vex::PORT11,vex::ratio6_1,true);
vex::motor motor_bl2=vex::motor(vex::PORT12,vex::ratio6_1,false);

vex::motor middle_motor1=vex::motor(vex::PORT4,vex::ratio6_1,true);

vex::motor under_motor1=vex::motor(vex::PORT6,vex::ratio6_1,false);

vex::motor trans_motor1=vex::motor(vex::PORT5,vex::ratio6_1,true);
vex::motor trans_motor2=vex::motor(vex::PORT2,vex::ratio6_1,false);
vex::motor trans_motor3=vex::motor(vex::PORT7,vex::ratio6_1,true);

vex::motor under_overhang_motor=vex::motor(vex::PORT10,vex::ratio6_1,false);

vex::motor up_overhang_motor=vex::motor(vex::PORT3,vex::ratio6_1,true);
vex::motor middle_overhang_motor=vex::motor(vex::PORT19,vex::ratio6_1,true);
vex::motor up_motor1=vex::motor(vex::PORT1,vex::ratio6_1,true);

vex::motor sensor=vex::motor(vex::PORT9,vex::ratio6_1,true);

vex::brain Brain; 

vex::controller Controller = vex::controller(vex::controllerType::primary);

vex::inertial Inertial = vex::inertial(vex::PORT20); 
