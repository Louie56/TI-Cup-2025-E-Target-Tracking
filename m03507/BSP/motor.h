#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "board.h"

extern int targetx_steps;  //X 目标步数
extern int currentx_steps;   //X 当前走了多少步

extern int targety_steps;  // Y 目标步数
extern int currenty_steps;//Y 当前走了多少步
extern int last_delay_us_val;


#define CW() DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_3)//顺时针
#define ACW() DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_3)//逆时针

void move_motorx_steps(int steps);//1逆0顺
void move_motory_steps(int steps);

void Motor1(int errorx,int errory);
void Motortest(int errorx, int errory);

void TwoAxis_Move(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps);
void TwoAxis_Movetest(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps);
void TwoAxis_Move_Line(int target_x_steps, int target_y_steps, unsigned int target_speed_us);

void TwoAxis_Movetest_Continuous(int target_steps_x, int target_steps_y, int target_period_min);
void Motor_Stop_Smoothly() ;

	void TwoAxis_Movetest1(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps);


void movx(int dir);
void movy(int dir);

#endif 
