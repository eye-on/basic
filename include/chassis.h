#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "vex.h"

class Chassis{
private:

    //fl 左上电机速度 fr右上电机速度 bl左下电机速度 右下电机速度
    double fl=0 ,fr=0 ,bl=0 ,br=0;

    double fwd=0,rot=0;

    double a[5]={0,0,0,0,0};

    double FL=0,FR=0,BL=0,BR=0;

    //刹车模式 enum类型
    vex::brakeType stopBrakeType=vex::coast;

public:

    /** 
    *@brief 单例模式，用于保证Chassis类仅有一个实例，并提供一个全局全局访问点
    */
    static Chassis *getInstance(){
        static Chassis *c = nullptr;
        if (c == nullptr) {
            c = new Chassis();
        }
        return c;
    }

    /** 
    *@brief 删除Chassis的实例
    */
    static void deleteInstance() {
        Chassis *c = Chassis::getInstance();
        if (c != nullptr) {
            delete c;
            c = nullptr;
        }
    }

    /**
     * @brief 刹车函数
     * @param type 刹车类型
     */
    void Brake(vex::brakeType type);

    /** 
    *@brief 根据成员变量fl fr bl br设置电机速度
    */
    void Set_MotorPower();

    /** 
    *@brief  动态调整平滑系数：输入变化快（快速操作）时，降低平滑（减少延迟）；
             输入变化慢（精细操作）时，增加平滑（减少抖动）
    *@param now 当前帧速度
    *@param last 上一帧速度
    *@param rating 摇杆移动速度
    *@return 动态平滑系数
    */
    double dynamicSmooth(int now,int last,double rating);


    /** 
    *@brief 全向底盘控制，通过对x,y,rot三个分速度的合成控制电机速度
    */
    void Omni_chassiscontrol();
   
};
    /** 
    *@brief 该函数运行于独立线程，用于更新电机速度
    */
void chassis_updating_thread();

#endif
