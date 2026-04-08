#include "basic_functions.h"
#include "robot_confing.h"

#include <cmath>

void set_motorpower(vex::motor base,double speed,vex::brakeType type){
    if(speed){
        base.spin(vex::directionType::fwd,speed,vex::percentUnits::pct);
    }else{
        base.stop(type);
    }
}

double function(double input){
    bool sign =input<0;
    double min=std::abs(input)*0.01;
    double t=min*min*(3-2*min)*100.0;
    return (sign?-t:t);
}



/*void enable_motor_analog(vex::motor base,bool dir,int speed){
    if(dir){
        base.spin(vex::fwd, speed, vex::pct);
    }else{
        base.spin(vex::fwd, -1*speed, vex::pct);
    }
}*/

void enable_motor_analog(vex::motor base,int input,int deadzone,int speed){
    int dir= (input<0) ? -1:1;;
    bool sign=(abs(input) < deadzone) ? 0 : 1;
    if(sign){
        set_motorpower(base,dir*speed);
    }
    else{
        base.stop();
    }
}

/*void enable_motor_discrete(vex::motor base,bool dir,int speed){
    static bool motor_Start=false;
    if(!motor_Start){
        if(dir){
            base.spin(vex::fwd, speed, vex::pct);
        }
        else{
            base.spin(vex::fwd, -1*speed, vex::pct);
        }
        motor_Start=true;
    }else{
        base.stop();
        motor_Start=false;
    }
}*/

void enable_motor_discrete(vex::motor base,bool& input,int speed){
    static bool motor_Start=false;
    if(input){
        if(!motor_Start){
            base.spin(vex::fwd,speed,vex::pct);
            motor_Start=true;
        }else{
            base.stop();
            motor_Start=false;
        }
        input=false;
    }
}

void enable_motor_discrete(std::vector<vex::motor> motors,bool& input,std::vector<int> speeds){
    static bool motor_Start=false;
    if(input){
        if(!motor_Start){
            //base.spin(vex::fwd,dir*speed,vex::pct);
            int count=0;
            for(auto motor:motors){
                motor.spin(vex::fwd,speeds[count],vex::pct);
                count+=1;
            }
            motor_Start=true;
        }else{
            //base.stop();
            for(auto motor:motors){
                motor.stop();
            }
            motor_Start=false;
        }
        input=false;
    }
}

/* 传感器数据处理函数 */
// 获取IMU修正后的航向角（0-360度）
double IMUHeading() {
    double heading = Inertial.rotation(vex::rotationUnits::deg);  // 原始旋转角度
    // 应用校准系数（IMU_MOD_COEFFICIENT为预设校准参数）
    heading = heading / IMU_MOD_COEFFICIENT * 3600;
    // 角度规范化处理（确保在0-360度范围内）
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    return heading;
}

// 重置IMU航向角基准（归零操作）
void resetHeading() { Inertial.resetRotation(); }

double rad_to_deg(double rad) { return rad / M_PI * 180.0; }

double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
