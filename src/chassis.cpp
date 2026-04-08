#include "chassis.h"
#include "basic_functions.h"
#include "robot_confing.h"
#include "parameters.h"
#include "button.h"

#include <algorithm>

void Chassis::Brake(vex::brakeType type){
    //重置各轮速度
    fl=0;
    fr=0;
    bl=0;
    br=0;

    //强制停止所有电机
    motor_bl1.stop(type);
    motor_bl2.stop(type);
    motor_br1.stop(type);
    motor_br2.stop(type);
    motor_fl1.stop(type);
    motor_fl2.stop(type);
    motor_fr1.stop(type);
    motor_fr2.stop(type);
}

void Chassis::Set_MotorPower(){
    set_motorpower(motor_bl1,bl,stopBrakeType);
    set_motorpower(motor_bl2,bl,stopBrakeType);

    set_motorpower(motor_br1,br,stopBrakeType);
    set_motorpower(motor_br2,br,stopBrakeType);

    set_motorpower(motor_fl1,fl,stopBrakeType);
    set_motorpower(motor_fl2,fl,stopBrakeType);

    set_motorpower(motor_fr1,fr,stopBrakeType);
    set_motorpower(motor_fr2,fr,stopBrakeType);
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
    a[1]=dynamicSmooth(A1,last_A1,rating[0]);
    a[2]=dynamicSmooth(A2,last_A2,rating[1]);
    a[3]=dynamicSmooth(A3,last_A3,rating[2]);
    a[4]=dynamicSmooth(A4,last_A4,rating[3]);

    //合成速度
    FL=a[2]+a[1];
    FR=a[2]-a[1];
    BL=a[2]+a[1];
    BR=a[2]-a[1];

    //限制输出范围
    double maxpct=std::max({fabs(FL),fabs(FR),fabs(BL),fabs(BR)});
    if(maxpct>100){
        double k=100.0/maxpct;
        FL*=k;
        FR*=k;
        BL*=k;
        BR*=k;
    }
    
    //赋值各轮速度
    fl=function(FL);
    fr=function(FR);
    bl=function(BL);
    br=function(BR);
}

void chassis_updating_thread(){

    while(true){
        //调用单一实例
        Chassis::getInstance()->Set_MotorPower();

        // 线程休眠，控制更新频率
        vex::this_thread::sleep_for(REFRESH_TIME); 
    }
}
