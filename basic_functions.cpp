#include"basic_functions.h"
#include"robot_confing.h"

#include<cmath>

void set_motorpower(vex::motor motor,double speed,vex::brakeType type){
    if(speed){
        motor.spin(vex::fwd,speed,vex::rpm);
    }else{
        motor.stop(type);
    }
}

void set_motorpower(vex::motor_group motors,double speed,vex::brakeType type){
    if(speed){
        motors.spin(vex::fwd,speed,vex::rpm);
    }else{
        motors.stop(type);
    }
}

double function(double input){
    bool sign =input<0;
    double min=std::abs(input)*0.01;
    double t=min*min*(3-2*min)*100.0;
    return (sign?-t:t);
}

/* 传感器数据处理函数 */
// 获取IMU修正后的航向角（0-360度）
double IMUHeading() {
    double heading = Inertial.rotation(vex::rotationUnits::deg);  // 原始旋转角度
    // 应用校准系数（IMU_MOD_COEFFICIENT为预设校准参数）
    heading = heading / IMU_MOD_COEFFICIENT * 3600;
    // 角度规范化处理（确保在0-360度范围内）
    /*while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;*/
    return heading;
}

// 重置IMU航向角基准（归零操作）
void resetHeading() { Inertial.resetRotation(); }

double rad_to_deg(double rad) { return rad / M_PI * 180.0; }

double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

double clamp(double value, double min, double max) {
        if (value > max) return max;
        if (value < min) return min;
        return value;
}


