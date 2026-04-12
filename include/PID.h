#ifndef PID_H_
#define PID_H_

#include"parameters.h"

class PID{
private:
    double kp,ki,kd;
    double cur_value=0;
    double last_error=0,cur_error=0,sum_error=0,cur_d_error=0,last_d_error=0;
    double p_output=0,i_output=0,d_output=0,output=0;
    const double T=REFRESH_TIME_ms/1000.0/4;
public:
    double point_value;
    PID(double p,double i,double d,double value):kp(p),ki(i),kd(d),point_value(value){
    };
    double compute(double cur,double t);
    void reset();
};

#endif
