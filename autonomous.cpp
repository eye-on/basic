#include "autonomous.h"
#include "robot_confing.h"

void autonomous(){
  turn_param turn_1(0.5,0,0,160);
  turn_param turn_2(0.5,0,0,200);
  turn_param turn_3(1,0,0,160);
  turn_param turn_4(1,0,0,270);
  /*under_overhang_motor.spinFor(-480,vex::deg,50,vex::velocityUnits::pct);
  up_overhang_motor.spinFor(-350,vex::deg,50,vex::velocityUnits::pct);
  up_overhang_motor.spinFor(-200,vex::deg,50,vex::velocityUnits::pct);
  under_overhang_motor.stop(vex::hold);*/

  /*middle_overhang_motor.spinFor(520,vex::deg,50,vex::velocityUnits::pct,false);
  vex::this_thread::sleep_for(500);
  up_overhang_motor.spinFor(-900,vex::deg,50,vex::velocityUnits::pct,false);
  up_overhang_motor.spinFor(-700,vex::deg,50,vex::velocityUnits::pct);
  under_overhang_motor.spinFor(900,vex::deg,50,vex::velocityUnits::pct,false);
  vex::this_thread::sleep_for(500);*/

  
  vex::this_thread::sleep_for(3000);
  forward_ms(1525,200);
  //init();
  turn_deg(-90,turn_1);
  auto_takein();
  forward_ms(1850,150);
  vex::this_thread::sleep_for(3000);
  vex::this_thread::sleep_for(3000);
  auto_takein();
  vex::this_thread::sleep_for(1000);
  forward_ms(825,200,vex::directionType::rev);
  //under_overhang_motor.spinFor(-900,vex::deg,50,vex::velocityUnits::pct);
  uper_under_overhang();
  turn_deg(-90,turn_1);
  turn_deg(-90,turn_1);
  vex::this_thread::sleep_for(500);
  forward_ms(1450,100);
  vex::this_thread::sleep_for(500);
  //up_overhang_motor.spinFor(-100,vex::deg,30,vex::velocityUnits::pct);
  auto_up_push();
  vex::this_thread::sleep_for(6000);
  auto_up_push();
  forward_ms(800,150,vex::directionType::rev);
  vex::this_thread::sleep_for(500);
  turn_deg(90,turn_1);
  forward_ms(700,150);
  turn_deg(-90,turn_1);
  forward_ms(3500,100);

}
/*forward_ms(1400,200);
  turn_deg(60,turn_1);
  vex::this_thread::sleep_for(500);
  forward_ms(1400,200);
  under_overhang_motor.spinFor(480,vex::deg,50,vex::velocityUnits::pct);
  auto_takein();
  forward_ms(1000,50,vex::directionType::rev);
  forward_ms(500,100);
  vex::this_thread::sleep_for(1000);
  auto_takein();
  forward_ms(1200,200,vex::directionType::rev);
  turn_deg(-108,turn_2);
  vex::this_thread::sleep_for(500);
  under_overhang_motor.spinFor(-480,vex::deg,50,vex::velocityUnits::pct);
  forward_ms(1500,100);
  auto_under_push();
  vex::this_thread::sleep_for(3000);
  auto_under_push();
  forward_ms(2700,200,vex::directionType::rev);
  turn_deg(-137,turn_3);
  vex::this_thread::sleep_for(500);
  under_overhang_motor.spinFor(480,vex::deg,50,vex::velocityUnits::pct);
  auto_takein();
  forward_ms(3000,100);
  vex::this_thread::sleep_for(4000);
  auto_takein();
  forward_ms(1000,200,vex::directionType::rev);
  under_overhang_motor.spinFor(-900,vex::deg,50,vex::velocityUnits::pct);
  turn_deg(180,turn_4);
  vex::this_thread::sleep_for(500);
  forward_ms(100,200);
  auto_up_push();
  vex::this_thread::sleep_for(4000);
  auto_up_push();*/