#include"auto_functions.h"
#include"basic_functions.h"
#include"parameters.h"
#include"robot_confing.h"
#include"chassis.h"
#include"timer.h"
#include"position.h"
#include"pid.h"
#include<algorithm>

void shake(){
  chassis.stop(vex::coast);
  Timer f_time;
  f_time.reset();
  while(f_time.getTime_ms()<300){
    chassis.spin(vex::directionType::rev,50,vex::rpm);
  }
  f_time.reset();
  while(f_time.getTime_ms()<300){
    chassis.spin(vex::fwd,50,vex::rpm);
  }
  chassis.stop(vex::hold);
}

void forward_ms(double time_ms,double max_rpm,vex::directionType dir){
    chassis.stop(vex::coast);
    //最小加速时间 加速时间占比
    double min_acc_time_ms=300,acc_time_pct=0.3;
    //最小减速时间 减速时间占比
    double min_slow_time_ms=300,slow_time_pct=0.3;
    //实际加速,匀速,减速时间
    double acc_time_ms=0,uni_time_ms=0,slow_time_ms=0;
    double cur_rpm=0,delta=0;
    //时间计算
    acc_time_ms=std::max(time_ms*acc_time_pct,min_acc_time_ms);
    slow_time_ms=std::max(time_ms*slow_time_pct,min_slow_time_ms);
    uni_time_ms=time_ms-acc_time_ms-slow_time_ms;

    double k = 1;
    if(dir==vex::directionType::rev){
      k=-1;
    }
    // 记录初始位置（用于后续方向修正）
    /*Position::getInstance()->reset();
    double base_l = Position::getInstance()->get_lrad();
    double base_r = Position::getInstance()->get_rrad();*/
    double base_deg=Position::getInstance()->getIMU_deg();
    Timer f_time;
    f_time.reset();
    while(f_time.getTime_ms()<acc_time_ms){
        //delta_l_rad=Position::getInstance()->get_lrad()-base_l;
        //delta_r_rad=Position::getInstance()->get_rrad()-base_r;
        delta=Position::getInstance()->getIMU_deg()-base_deg;

        cur_rpm = max_rpm / acc_time_ms * f_time.getTime_ms(); 

        //right_motors.spin(dir,cur_rpm+kp*delta,vex::rpm);
        //left_motors.spin(dir,cur_rpm-kp*delta,vex::rpm);
        Chassis::getInstance()->left_speed=k*cur_rpm;
        Chassis::getInstance()->right_speed=k*cur_rpm;

        //chassis.spin(dir,cur_rpm,vex::rpm);


        //vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    //max_rpm=cur_rpm;
    f_time.reset();
    while(f_time.getTime_ms()<uni_time_ms){
        //delta_l_rad=Position::getInstance()->get_lrad()-base_l;
        //delta_r_rad=Position::getInstance()->get_rrad()-base_r;
        delta=Position::getInstance()->getIMU_deg()-base_deg;
        //right_motors.spin(dir,cur_rpm+kp*delta,vex::rpm);
        //left_motors.spin(dir,cur_rpm-kp*delta,vex::rpm);

        //chassis.spin(dir,cur_rpm,vex::rpm);

        Chassis::getInstance()->left_speed=k*cur_rpm;
        Chassis::getInstance()->right_speed=k*cur_rpm;

        //vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    f_time.reset();
    while(f_time.getTime_ms()<slow_time_ms){
        //delta_l_rad=Position::getInstance()->get_lrad()-base_l;
        //delta_r_rad=Position::getInstance()->get_rrad()-base_r;
        delta=Position::getInstance()->getIMU_deg()-base_deg;
        cur_rpm = max_rpm-(max_rpm / slow_time_ms * f_time.getTime_ms());

        //right_motors.spin(dir,cur_rpm+kp*delta,vex::rpm);
        //left_motors.spin(dir,cur_rpm-kp*delta,vex::rpm);

        //chassis.spin(dir,cur_rpm,vex::rpm);

        Chassis::getInstance()->left_speed=k*cur_rpm;
        Chassis::getInstance()->right_speed=k*cur_rpm;

        //vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    chassis.stop(vex::hold);
}

void backward_ms(double time_ms,double max_rpm){
    chassis.stop(vex::hold);
    //最小加速时间 加速时间占比
    double min_acc_time_ms=200,acc_time_pct=0.2;
    //最小减速时间 减速时间占比
    double min_slow_time_ms=200,slow_time_pct=0.4;
    //实际加速,匀速,减速时间
    double acc_time_ms=0,uni_time_ms=0,slow_time_ms=0;
    double cur_rpm=0,delta=0;
    //时间计算
    acc_time_ms=std::max(time_ms*acc_time_pct,min_acc_time_ms);
    slow_time_ms=std::max(time_ms*slow_time_pct,min_slow_time_ms);
    uni_time_ms=time_ms-acc_time_ms-slow_time_ms;

    double kp = 0.1;
    // 记录初始位置（用于后续方向修正）
    /*Position::getInstance()->reset();
    double base_l = Position::getInstance()->get_lrad();
    double base_r = Position::getInstance()->get_rrad();
*/
    Inertial.resetHeading();
    double base_deg=Position::getInstance()->getIMU_deg();

    Timer f_time;
    f_time.reset();
    while(f_time.getTime_ms()<acc_time_ms){
        delta=Position::getInstance()->getIMU_deg()-base_deg;

        cur_rpm = max_rpm / acc_time_ms * f_time.getTime_ms();

        left_motors.spin(vex::directionType::rev,cur_rpm-kp*delta,vex::rpm);
        right_motors.spin(vex::directionType::rev,cur_rpm+kp*delta,vex::rpm);

        vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    max_rpm=cur_rpm;
    f_time.reset();
    while(f_time.getTime_ms()<uni_time_ms){
        delta=Position::getInstance()->getIMU_deg()-base_deg;

        //chassis.spin(vex::fwd,max_rpm,vex::rpm);
        left_motors.spin(vex::directionType::rev,cur_rpm-kp*delta,vex::rpm);
        right_motors.spin(vex::directionType::rev,cur_rpm+kp*delta,vex::rpm);

        vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    f_time.reset();
    while(f_time.getTime_ms()<slow_time_ms){
        delta=Position::getInstance()->getIMU_deg()-base_deg;

        cur_rpm = max_rpm-(max_rpm / slow_time_ms * f_time.getTime_ms());

        left_motors.spin(vex::directionType::rev,cur_rpm-kp*delta,vex::rpm);
        right_motors.spin(vex::directionType::rev,cur_rpm+kp*delta,vex::rpm);

        vex::this_thread::sleep_for(REFRESH_TIME_ms / 2);
    }
    chassis.stop(vex::hold);
}

void turn_deg(double point_deg,const turn_param& param){
    chassis.stop(vex::coast);
    PID pid(param.Kp,param.Ki,param.Kd,point_deg);
    double max_output=-100000,min_output=100000;
    int limit_rpm=param.rpm_range;
    //Inertial.resetHeading();
    double base_deg=Position::getInstance()->getIMU_deg();
    /*double x=pid.compute(0,REFRESH_TIME_ms)>0;
    if(x>0){
      max_output=x;
      min_output=-x;
    }else{
      max_output=-x;
      min_output=x;
    }*/
    /*min_output=pid.compute(0);
    max_output=-0.2*min_output;*/
    Timer t_time;
    t_time.reset();
    double time_s=t_time.getTime_ms();
    for(double cur_deg=0;fabs(cur_deg-point_deg)>0.1;){
      cur_deg=(Position::getInstance()->getIMU_deg()-base_deg);
      double t=(t_time.getTime_ms()-time_s)*0.001;
      double output=pid.compute(cur_deg,t);
      max_output=std::max(output,max_output);
      min_output=std::min(output,min_output);
      
      double cur_rpm;
      if(output>0){
        cur_rpm=output*limit_rpm/max_output;
      }
      if(output<0){
        cur_rpm=-output*limit_rpm/min_output;
      }
      if(fabs(cur_rpm)<5){
        if(cur_rpm>0){
            cur_rpm=5;
        }else if(cur_rpm<0){
            cur_rpm=-5;
        }else{
            cur_rpm=0;
        }
      }
      
      Chassis::getInstance()->left_speed=-cur_rpm;
      Chassis::getInstance()->right_speed=cur_rpm;

      //left_motors.spin(vex::fwd,-cur_rpm,vex::rpm);
      //right_motors.spin(vex::fwd,cur_rpm,vex::rpm);

      //vex::this_thread::sleep_for(REFRESH_TIME_ms/4);
    }
    chassis.stop(vex::hold);
}

void auto_takein(){
  static bool started=false;
  started=!started;
  if(started){
    other_motors.stop(vex::hold);
    under_motor1.spin(vex::fwd,-100,vex::pct);
    middle_motor1.spin(vex::fwd,80,vex::pct);
    up_motor1.spin(vex::fwd,-80,vex::pct);
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

void auto_under_push(){
  static bool started=false;
  started=!started;
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

void auto_middle_push(){
  static bool started=false;
    started=!started;
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

void auto_up_push(){
  static bool started=false;
    started=!started;
    if(started){
      other_motors.stop(vex::hold);
      trans_motor1.spin(vex::fwd,-100,vex::pct);
      trans_motor2.spin(vex::fwd,100,vex::pct);
      trans_motor3.spin(vex::fwd,-100,vex::pct);
      trans_motor4.spin(vex::fwd,50,vex::pct);
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

void uper_under_overhang(){
  wait(1000,vex::msec);
  under_overhang_motor.spin(vex::fwd,-50,vex::pct);
  wait(600,vex::msec);
  under_overhang_motor.stop(vex::hold);
}