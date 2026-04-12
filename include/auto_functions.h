#ifndef AUTO_FUNCTIONS_H_
#define AUTO_FUNCTIONS_H_

#include "vex.h"
#include "pid.h"

class turn_param{
public:
    double Kp,Ki,Kd;
    int rpm_range;
    turn_param(double kp,double ki,double kd,int range=600):Kp(kp),Ki(ki),Kd(kd),rpm_range(range){}
};

void shake();

void forward_ms(double time_ms,double max_rpm=300,vex::directionType dir=vex::fwd);

void backward_ms(double time_ms,double max_rpm=300);

void turn_deg(double point_deg,const turn_param& param);

void auto_takein();

void auto_under_push();

void auto_middle_push();

void auto_up_push();

void uper_under_overhang();

#endif
