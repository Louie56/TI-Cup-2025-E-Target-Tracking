#include "time.h"


int error_x,error_y;
int KX=0,KX_flag,KY=0,KY_flag;
int speedx,speedy;

void TIMG0_Init(void)
{
	NVIC_EnableIRQ(TIMG0_INT_IRQn);//使能
	DL_TimerG_startCounter(TIMG0);
}

void TIMG8_Init(void)
{
	NVIC_EnableIRQ(TIMG8_INT_IRQn);//使能
	DL_TimerG_startCounter(TIMG8);
}



void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {	
        DL_Timer_clearInterruptStatus(TIMG0,DL_TIMERG_IIDX_ZERO);//清除标志位



		
		
    }
}

void TIMG8_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_1_INST))
    {
        DL_Timer_clearInterruptStatus(TIMG8,DL_TIMERG_IIDX_ZERO);//清除标志位
		
    }
}
int my_abs(int a)
{
	if(a<0) a=-a;
	return a;
}
void Openmv_Data(void)
{
	extern int x_err,y_err;
	KX=my_abs(x_err);
	KX_flag= (x_err>=0?1:0);
	KY=my_abs(y_err);
	KY_flag= (y_err>=0?1:0);
	speedx=KX_flag;
	speedy=KY_flag;
	

	
////	if(KX<2&&KX>0)
//	if(KX==0)
//	{
//		speedx=0;
//	}
//	else 
//	{
//		if(KX_flag==1)speedx=1;
//		else speedx=-1;
//	}
//	//if(KY<2&&KY>0)
//	if(KY==0)
//	{
//		speedy=0;
//	}
//	else 
//	{
//		if(KY_flag==1)speedy=-1;
//		else speedy=1; 
//	}
}



