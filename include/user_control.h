#ifndef USERCONTROL_H_
#define USERCONTROL_H_

/** 
*@brief 基础控制
*/
void base_control();

void transmotor_control();

void high_out();

void under_inout();

void under_overhang_control();

void middle_overhang_control();

void up_overhang_control();

void eatin_ball();

void old_eatin_ball();

void up_throw_out();

void middle_throw_out();

void stop_all();

/** 
*@brief 该函数运行于独立线程，用于响应手柄操作
*/
void user_control_thread();

/** 
*@brief 运行用户操作的独立线程
*/
void user_control();

#endif

