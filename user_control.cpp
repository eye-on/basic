#include"user_control.h"
#include"button.h"
#include"chassis.h"
#include"parameters.h"
#include"basic_functions.h"
#include"robot_confing.h"
#include"sensor.h"
#include"autonomous.h"

#include<cmath>
#include<algorithm>

void base_control() {

  Chassis::getInstance()->Omni_chassiscontrol();

}

void takein(){
  static bool started=false;
  if(press_A){
    started=!started;
    press_A=false;
    if(started){
      other_motors.stop(vex::hold);
      under_motor1.spin(vex::fwd,-100,vex::pct);
      middle_motor1.spin(vex::fwd,80,vex::pct);
      up_motor1.spin(vex::fwd,-70,vex::pct);
      trans_motor3.spin(vex::fwd,-100,vex::pct);
      trans_motor4.spin(vex::fwd,50,vex::pct);
  }else{
      under_motor1.stop(vex::hold);
      middle_motor1.stop(vex::hold);
      up_motor1.stop(vex::hold);
      trans_motor3.stop(vex::hold);
      trans_motor4.stop(vex::hold);
    }
  }
}

void under_push(){
  static bool started=false;
  if(press_B){
    started=!started;
    press_B=false;
    if(started){
      other_motors.stop(vex::hold);
      under_motor1.spin(vex::fwd,100,vex::pct);
      trans_motor1.spin(vex::fwd,-100,vex::pct);
      trans_motor2.spin(vex::fwd,100,vex::pct);
      trans_motor3.spin(vex::fwd,-100,vex::pct);
  }else{        
      under_motor1.stop(vex::hold);
      trans_motor1.stop(vex::hold);
      trans_motor2.stop(vex::hold);
      trans_motor3.stop(vex::hold);
    }
  }
}

void middle_push(){
  static bool started=false;
  if(press_Y){
    started=!started;
    press_Y=false;
    if(started){
      other_motors.stop(vex::hold);
      trans_motor1.spin(vex::fwd,-100,vex::pct);
      trans_motor2.spin(vex::fwd,100,vex::pct);
      trans_motor3.spin(vex::fwd,-100,vex::pct);
      trans_motor4.spin(vex::fwd,100,vex::pct);
      under_motor1.spin(vex::fwd,-100,vex::pct);
      middle_motor1.spin(vex::fwd,-100,vex::pct);
  }else{
      trans_motor1.stop(vex::hold);
      trans_motor2.stop(vex::hold);
      trans_motor3.stop(vex::hold);
      trans_motor4.stop(vex::hold);
      under_motor1.stop(vex::hold);
      middle_motor1.stop(vex::hold);
    }
  }
}

void up_push(){
  static bool started=false;
  if(press_X){
    started=!started;
    press_X=false;
    if(started){
      other_motors.stop(vex::hold);
      trans_motor1.spin(vex::fwd,-100,vex::pct);
      trans_motor2.spin(vex::fwd,100,vex::pct);
      trans_motor3.spin(vex::fwd,-100,vex::pct);
      trans_motor4.spin(vex::fwd,100,vex::pct);
      under_motor1.spin(vex::fwd,-100,vex::pct);
      middle_motor1.spin(vex::fwd,100,vex::pct);
      up_motor1.spin(vex::fwd,100,vex::pct);
    }else{
      trans_motor1.stop(vex::hold);
      trans_motor2.stop(vex::hold);
      trans_motor3.stop(vex::hold);
      trans_motor4.stop(vex::hold);
      under_motor1.stop(vex::hold);
      middle_motor1.stop(vex::hold);
      up_motor1.stop(vex::hold);
    }
  }
}

void under_overhang_control(){
  if(UP){
    under_overhang_motor.spin(vex::fwd,-20,vex::pct);
  }else if(DOWN){
    under_overhang_motor.spin(vex::fwd,20,vex::pct);
  }else{
    under_overhang_motor.stop(vex::hold);
  }
}

void middle_overhang_control(){
  if(LEFT){
    middle_overhang_motor.spin(vex::fwd,20,vex::pct);
  }else if(RIGHT){
    middle_overhang_motor.spin(vex::fwd,-20,vex::pct);
  }else{
    middle_overhang_motor.stop(vex::hold);
  }
}

void up_overhang_control(){
  if(L1){
    up_overhang_motor.spin(vex::fwd,20,vex::pct);
  }else if(L2){
    up_overhang_motor.spin(vex::fwd,-20,vex::pct);
  }else{
    up_overhang_motor.stop(vex::hold);
  }
}

void solve(){
  if(press_R2){
    if(under_motor1.isSpinning()){
    under_motor1.stop(vex::hold);
    }
    if(middle_motor1.isSpinning()){
      middle_motor1.stop(vex::hold);
    }
    if(up_motor1.isSpinning()){
      up_motor1.stop(vex::hold);
    }
    if(trans_motor1.isSpinning()){
      trans_motor1.stop(vex::hold);
    }
    if(trans_motor2.isSpinning()){
      trans_motor2.stop(vex::hold);
    }
    if(trans_motor3.isSpinning()){
      trans_motor3.stop(vex::hold);
    }
    if(trans_motor4.isSpinning()){
      trans_motor4.stop(vex::hold);
    }
    if(under_overhang_motor.isSpinning()){
      under_overhang_motor.stop(vex::hold);
    }
    if(middle_overhang_motor.isSpinning()){
      middle_overhang_motor.stop(vex::hold);
    }
    if(up_overhang_motor.isSpinning()){
      up_overhang_motor.stop(vex::hold);
    }
    press_R2=false;
  }
}

void user_control_thread(){
  while(true){
    base_control();
    up_push();
    middle_push();
    under_push();
    takein();
    up_overhang_control();
    under_overhang_control();
    middle_overhang_control();
    //solve();
    //vex::this_thread::sleep_for(REFRESH_TIME_ms);
  }
}

void user_control(){

  static vex::thread UsrCtl(user_control_thread);

}
