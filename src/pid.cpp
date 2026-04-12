#include"pid.h"
#include"basic_functions.h"
#include<algorithm>
#include"robot_confing.h"

double PID::compute(double cur,double t){
    //计算误差
    cur_error=cur-point_value;
    double k=1-fabs(cur_error/point_value);
    //p项
    p_output=kp*cur_error;
    //i项
    sum_error+=cur_error*t;
    if(fabs(sum_error)>600.0/ki)
    sum_error=0;
    i_output=ki*sum_error;
    //d项
    cur_d_error=0.8*k*(cur_error-last_error)/t+(1.0-0.8*k)*last_d_error;
    d_output=kd*cur_d_error;
    last_d_error=cur_d_error;
    //p i d加和输出
    output=p_output+i_output+d_output;

    last_error=cur_error;

    return p_output;
}

void PID::reset(){
  cur_error=0;
  last_error=0;
  sum_error=0;
  cur_d_error=0;
  last_d_error=0;
  p_output=0;
  i_output=0;
  d_output=0;
  output=0;
}
