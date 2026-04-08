#ifndef BUTTON_H_
#define BUTTON_H_

extern int 
        t;
        
extern int 
        A1, A2, A3, A4,
        last_A1,last_A2,last_A3,last_A4;

extern bool 
        L1, L2, R1, R2, 
        X, Y, A, B, 
        LEFT, RIGHT, UP, DOWN, 
        last_L1, last_L2, last_R1, last_R2,
        last_X, last_Y, last_A, last_B, 
        last_LEFT, last_RIGHT, last_UP, last_DOWN;

extern bool 
        press_X, press_Y, press_A, press_B, press_UP, press_DOWN, press_LEFT, press_RIGHT, 
        press_L1, press_L2,press_R1, press_R2;

extern bool 
        praise_X,praise_Y,praise_A,praise_B,praise_UP,praise_DOWN,praise_LEFT,praise__RIGHT,
        praise_L1,praise_L2,praise_R1,praise_R2;

extern double rating[4];



/** 
 * @brief 根据每帧时间和按键移动距离计算按键移动速率 [0,1]
 * @return 按键速度
 */
void calcu_buttonrating();

/** 
 * @brief 该函数运行于独立线程中，用于更新按键状态
 * 
 */
void button_updating_thread();

#endif