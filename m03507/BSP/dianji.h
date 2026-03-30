#ifndef _DIANJI_H_
#define _DIANJI_H_

#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

#include "dianji.h"

int one(int a);
extern	No_MCU_Sensor sensor;

void go_straight(void);
int get_ADs(int a);
void Stop(void);
int max(int speed);
int get_lines(void);
void Beep_LED_ON(void);
void Beep_LED_OFF(void);
void Run_PID_LineFollowing(void) ;
void Set_Motor_Speeds(int steering_correction);

#endif
