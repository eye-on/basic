#include "user_control.h"
#include "button.h"
#include "chassis.h"
#include "parameters.h"
#include "basic_functions.h"
#include "robot_confing.h"
#include "sensor.h"

#include <algorithm>
#include <cmath>

 void base_control() {

    Chassis::getInstance()->Omni_chassiscontrol();

 }

void transmotor_control(){
      enable_motor_discrete(trans_motor1,press_A);
      enable_motor_discrete(trans_motor2,press_B);
      enable_motor_discrete(trans_motor3,press_X);
 }

 void high_out(){
      enable_motor_discrete(middle_motor1,press_L1);
      enable_motor_discrete(middle_motor1,press_L2);
 }

void under_inout(){

   enable_motor_analog(under_motor1,A3,DEAD_ZONE);

 }

 void under_overhang_control(){
   /*static bool position_up=false;
   if(press_Y){
      if(position_up){
         //overhang_motor.spinTo(90,vex::3wwsssseg,false);
         under_overhang_motor.spinFor(360,vex::deg,false);
         position_up=false;
      }else{
         //overhang_motor.spinTo(90,vex::deg,false);
         under_overhang_motor.spinFor(-360,vex::deg,false);
         position_up=true;
      }
      press_Y=false;
   }*/
   /*if(press_UP){
      under_overhang_motor.spinFor(-90,vex::deg,60,vex::velocityUnits::pct,false);
      press_UP=false;
   }
   if(press_DOWN){
      under_overhang_motor.spinFor(60,vex::deg,60,vex::velocityUnits::pct,false);
      press_DOWN=false;
   }*/
  if(UP){
   under_overhang_motor.spin(vex::fwd,-20,vex::pct);
  }
  if(DOWN){
   under_overhang_motor.spin(vex::fwd,20,vex::pct);
  }
  if(!UP&&!DOWN){
   under_overhang_motor.stop(vex::hold);
  }
 }

 void middle_overhang_control(){
   /*if(press_LEFT){
      middle_overhang_motor.spinFor(80,vex::deg,20,vex::velocityUnits::pct,false);
      press_LEFT=false;
   }
   if(press_RIGHT){
      middle_overhang_motor.spinFor(-80,vex::deg,20,vex::velocityUnits::pct,false);
      press_RIGHT=false;
   }*/
   if(LEFT){
      middle_overhang_motor.spin(vex::fwd,20,vex::pct);
   }
   if(RIGHT){
      middle_overhang_motor.spin(vex::fwd,-20,vex::pct);
   }
   if(!LEFT&&!RIGHT){
      middle_overhang_motor.stop(vex::hold);
   }
 }

 void   up_overhang_control(){
   /*static bool position_up=false;if(press_B){
      if(position_up){
         up_overhang_motor.spinFor(3600,vex::deg,false);
      }else{
         up_overhang_motor.spinFor(-3600,vex::deg,false);
      }
      press_X=false;
   }*/
   /*if(press_X){
      up_overhang_motor.spinFor(1440,vex::deg,false);
      press_X=false;
   }
   if(press_B){
      up_overhang_motor.spinFor(-1440,vex::deg,false);
      press_B=false;
   }*/
   if(X){
      up_overhang_motor.spin(vex::fwd,50,vex::pct);
   }
   if(B){
      up_overhang_motor.spin(vex::fwd,-50,vex::pct);
   }
   if(!X&&!B){
      up_overhang_motor.stop(vex::hold);
   }
 }

 void stop_all(){
   if(press_R1){
      trans_motor1.stop();
      trans_motor2.stop();
      trans_motor3.stop();
      middle_motor1.stop();
      under_motor1.stop();
      press_R1=false;
   }
 }

 void eatin_ball(){
   /*int k=Accelerate==1?100:(Accelerate==-1?0:0);
   std::vector<vex::motor> motors={trans_motor2,trans_motor3,under_motor1,middle_motor1,up_motor1};
   std::vector<int> speeds={45,45,100,45,Accelerate==1?100:(Accelerate==-1?0:0)};
   enable_motor_discrete(motors,press_R2,speeds);
   */
 }

 void old_eatin_ball(){
   std::vector<vex::motor> motors={trans_motor2,trans_motor3,under_motor1,middle_motor1,up_motor1};
   std::vector<int> speeds={45,30,100,45,30};
   enable_motor_discrete(motors,press_A,speeds);
 }

 void up_throw_out(){
   static std::vector<vex::motor> motors={trans_motor1,trans_motor2,trans_motor3,under_motor1,middle_motor1,up_motor1};
   static std::vector<int> speeds={45,45,45,100,45,-50};

   enable_motor_discrete(motors,press_R1,speeds);
 }

 void middle_throw_out(){
   static std::vector<vex::motor> motors={trans_motor1,trans_motor2,trans_motor3,under_motor1,middle_motor1};
   static std::vector<int> speeds={45,45,45,100,-60};

   enable_motor_discrete(motors,press_L1,speeds);
 }

 void user_control_thread(){
    while(true){
      //启用基础控制
      base_control();
      up_overhang_control();
      middle_overhang_control();
      under_overhang_control();

      old_eatin_ball();
      eatin_ball();
      middle_throw_out();
      up_throw_out();

      // 线程休眠，控制更新频率
      vex::this_thread::sleep_for(REFRESH_TIME);
    }
 }

 void user_control(){

   static vex::thread UsrCtl(user_control_thread);

 }
