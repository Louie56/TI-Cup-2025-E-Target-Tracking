#include "stdio.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "control.h"

#define speedA 690//640
#define speedB 690
//1.616比率
int speedsetA,speedsetB,basespeed;
int v_left,v_right;
int sum;

int Beep_cnt;



//void Beep_LED_ON(void)
//{
//	DL_GPIO_setPins(Notice_PORT,Notice_LS_PIN_PIN);  //LED控制输出高电平
//	Beep_cnt=15;
//}

//void Beep_LED_OFF()
//{
//	DL_GPIO_clearPins(Notice_PORT,Notice_LS_PIN_PIN);  //LED控制输出低电平
//}





//限幅
int max(int speed)
{
	if(speed>1000)
		return 1000;
	if(speed<350)
		return 350;//300
	else
		return speed;
}


//停止
void Stop(void)
{
	AIN1_OUT(1);
	AIN2_OUT(1);
	BIN1_OUT(1);
	BIN2_OUT(1);
}

int flag=0;



void go(int bias)
{
	float ka=5;
	
	v_left=max(basespeed+bias*ka);
	v_right=max(basespeed-bias*ka);
	if (v_left>0)
		AO_Control(1,v_left);
	if(v_left<0)
		AO_Control(1,-v_left);
	delay_ms(50);

	if(v_right>0)
		BO_Control(1,v_right);
	if(v_right<0)
		BO_Control(1,-v_right);
	delay_ms(50);

}




