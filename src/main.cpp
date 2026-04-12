/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       86135                                                     */
/*    Created:      2025/9/10 17:33:24                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include"robot_confing.h"
#include"chassis.h"
#include"button.h"
#include"user_control.h"
#include"parameters.h"
#include"sensor.h"
#include"autonomous.h"
#include"position.h"

using namespace std;
using namespace vex;

 #ifdef COMPETITION
 competition Competition;
 #endif

int main() {
    const bool Pitch_control = false;
    timer time_begin;          // 创建计时器对象
    time_begin.system();       // 记录系统启动时间（用于时间基准）

    // IMU传感器校准流程
    Inertial.calibrate();                  // 启动惯性测量单元校准
    waitUntil(!Inertial.isCalibrating());  // 阻塞直到校准完成
    Inertial.resetHeading();
    Inertial.resetRotation();


    Controller.Screen.setCursor(5, 1);     // 设置控制器屏幕光标位置
    Controller.Screen.print("      calibrated!"); // 显示校准完成提示

    // 启动多线程任务
    //thread Sensor(runsensor);
    thread Tposition(updatePosition);  // 位置追踪更新线程
    thread Tcontroller(button_updating_thread);  //控制器更新线程

    if(!Pitch_control){
        thread Tchassis(chassis_updating_thread);    // 底盘状态更新线程
    }
    
    //init();

    #ifdef COMPETITION
    Competition.autonomous(autonomous); // 设置自动运行函数
    Competition.drivercontrol(user_control); // 设置手动控制函数
    #endif

    
    while (true) {
        Brain.Screen.setCursor(1, 1);        // 设置屏幕光标位置
        Brain.Screen.print("IMUHeading: %.4f", IMUHeading()); // 打印当前航向角（保留4位小数）
        /*Controller.Screen.setCursor(1, 1);     // 设置控制器屏幕光标位置
        Controller.Screen.print("IMUHeading: %.4f", IMUHeading()); // 显示校准完成提示*/
        this_thread::sleep_for(REFRESH_TIME_ms); // 按预设刷新率暂停（防止过度刷新）
    }
    return 0;
}
