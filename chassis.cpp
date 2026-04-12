#include"chassis.h"
#include"basic_functions.h"
#include"robot_confing.h"
#include"parameters.h"
#include"button.h"

#include<algorithm>

void Chassis::Brake(vex::brakeType type){
    //重置各轮速度
    left_speed=0;
    right_speed=0;

    //强制停止所有电机
    chassis.stop(stopBrakeType);
}

void Chassis::Set_MotorPower(){
    set_motorpower(left_motors,left_speed);
    set_motorpower(right_motors,right_speed);
}

double Chassis::dynamicSmooth(int now,int last,double rating){
    if(std::abs(now)>DEAD_ZONE){
        double k = 0.4 + 0.5*rating;
        return now*k+last*(1-k);
    }else{
        double k = 0.7 + 0.2*rating;
        return last*(1-k);
    }
}

void Chassis::Omni_chassiscontrol(){

    //调用动态平滑函数
    /*a[1]=dynamicSmooth(A1,last_A1,rating[0]);
    a[2]=dynamicSmooth(A2,last_A2,rating[1]);
    a[3]=dynamicSmooth(A3,last_A3,rating[2]);
    a[4]=dynamicSmooth(A4,last_A4,rating[3]);*/

    l=A2+0.5*A1;
    r=A2-0.5*A1;
    double k1=1,k2=1;
    //限制输出范围
    double maxpct=std::max(fabs(l),fabs(r));
    if(maxpct>100){
        double k=100.0/maxpct;
        l*=k;
        r*=k;
    }
    if(l<0){
        k1=-1;
    }
    if(r<0){
        k2=-1;
    }
    l=k1*l*l*0.01*6;
    r=k2*r*r*0.01*6;
    //赋值各轮速度
    left_speed=l;
    right_speed=r;
}

void chassis_updating_thread(){

    while(true){
        //调用单一实例
        Chassis::getInstance()->Set_MotorPower();

        // 线程休眠，控制更新频率
        //vex::this_thread::sleep_for(REFRESH_TIME_ms);
    }
}
