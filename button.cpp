#include"button.h"
#include"parameters.h"
#include"basic_functions.h"
#include"robot_confing.h"

#include<cmath>


int
    t;
int
    A1, A2, A3, A4,
    last_A1,last_A2,last_A3,last_A4;
bool
    L1, L2, R1, R2,
    X, Y, A, B,
    LEFT, RIGHT, UP, DOWN,
    last_L1, last_L2, last_R1, last_R2,
    last_X, last_Y, last_A, last_B,
    last_LEFT, last_RIGHT, last_UP, last_DOWN;
bool
    press_X=false, press_Y=false, press_A=false, press_B=false,
    press_UP=false, press_DOWN=false, press_LEFT=false, press_RIGHT=false,
    press_L1=false, press_L2=false,press_R1=false, press_R2=false;

double rating[4]={0,0,0,0};

void calcu_buttonrating(){
    rating[0] = std::abs(A1-last_A1)*0.005;
    rating[1] = std::abs(A2-last_A2)*0.005;
    rating[2] = std::abs(A3-last_A3)*0.005;
    rating[3] = std::abs(A4-last_A4)*0.005;
}

void button_updating_thread(){
    while(true){
        // 无限循环，持续更新控制器状态
        // 保存上一帧的按钮状态（用于后续比较）

        last_A1 = A1;
        last_A2 = A2;
        last_A3 = A3;
        last_A4 = A4;
        last_L1 = L1;
        last_L2 = L2;
        last_R1 = R1;
        last_R2 = R2;
        last_X = X;
        last_Y = Y;
        last_A = A;
        last_B = B;
        last_LEFT = LEFT;
        last_RIGHT = RIGHT;
        last_UP = UP;
        last_DOWN = DOWN;


        //通过遥控器获取当前时间 (毫秒单位 ms)
        t=Brain.timer(vex::timeUnits::msec);

        // 读取摇杆轴位置（百分比单位 pct）
        A1 = Controller.Axis1.position(vex::percentUnits::pct);
        A2 = Controller.Axis2.position(vex::percentUnits::pct);
        A3 = Controller.Axis3.position(vex::percentUnits::pct);
        A4 = Controller.Axis4.position(vex::percentUnits::pct);

        //计算A1,A2,A3,A4按键速度 保存至rating[1-4]
        calcu_buttonrating();

        // 读取当前按钮状态（布尔值 bool：true=按下，false=释放）
        L1 = Controller.ButtonL1.pressing();
        L2 = Controller.ButtonL2.pressing();
        R1 = Controller.ButtonR1.pressing();
        R2 = Controller.ButtonR2.pressing();
        X = Controller.ButtonX.pressing();
        Y = Controller.ButtonY.pressing();
        A = Controller.ButtonA.pressing();
        B = Controller.ButtonB.pressing();
        LEFT = Controller.ButtonLeft.pressing();
        RIGHT = Controller.ButtonRight.pressing();
        UP = Controller.ButtonUp.pressing();
        DOWN = Controller.ButtonDown.pressing();

        // 检测按钮按下事件（当前帧按下且上一帧未按下）
        // 设置对应标志位为true（单次触发，需外部重置）
        if (X && !last_X) press_X = true; //if (!X && last_X) praise_X = true;
        if (A && !last_A) press_A = true; //if (!A && last_A) praise_A = true;
        if (B && !last_B) press_B = true; //if (!B && last_B) praise_B = true;
        if (Y && !last_Y) press_Y = true; //if (!Y && last_Y) praise_Y = true;
        if (UP && !last_UP) press_UP = true; //if (!UP && last_UP) praise_UP = true;
        if (DOWN && !last_DOWN) press_DOWN = true; //if (!DOWN && last_DOWN) praise_DOWN = true;
        if (RIGHT && !last_RIGHT) press_RIGHT = true; //if (!RIGHT && last_RIGHT) praise_RIGHT = true;
        if (LEFT && !last_LEFT) press_LEFT = true; //if (!LEFT && last_LEFT) praise_LEFT = true;
        if (L1 && !last_L1) press_L1 = true; //if (!L1 && last_L1) praise_L1 = true;
        if (L2 && !last_L2) press_L2 = true; //if (!L2 && last_L2) praise_L2 = true;
        if (R1 && !last_R1) press_R1 = true; //if (!R1 && last_R1) praise_R1 = true;
        if (R2 && !last_R2) press_R2 = true; //if (!R2 && last_R2) praise_R2 = true;

        // 线程休眠，控制更新频率
        vex::this_thread::sleep_for(REFRESH_TIME_ms);
    }
}
