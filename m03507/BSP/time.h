#ifndef _TIME_H
#define _TIME_H

#include "board.h"
#include "motor.h"

extern int error_x,error_y;
extern int KX,KX_flag,KY,KY_flag;
extern int speedx,speedy;
int my_abs(int a);

void TIMG0_Init(void);
void TIMG8_Init(void);

void TIMG0_IRQHandler(void);

void Openmv_Data(void);
#endif