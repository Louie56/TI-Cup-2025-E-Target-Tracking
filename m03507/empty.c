
#include "board.h"
#include "stdio.h"
#include "time.h"
#include "motor.h"
#include "control.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dianji.h"
#include "bsp_tb6612.h"
#include "key.h"
#include "time.h"
No_MCU_Sensor sensor;

unsigned short Anolog[8]={0};
unsigned short white[8]={2822,2812,2804,2605,2765,2766,2821,2725};
unsigned short black[8]={734,994,1139,445,665,460,826,671};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
extern int error_x,error_y;
extern int step_countx;
extern int KX;
int tar;
	
int step;
int kpx=1,kpy=1;
float e_k2 = 0.0f;
float integral = 0.0f;
int e_k1 = 0;  // 上一次误差


//extern int32_t Ex_cnt_right ;
//extern int32_t Ex_cnt_left;

extern int triangle_path_len;
extern No_MCU_Sensor sensor;
int Y=0;
int X=0;
    unsigned short Normal[8];
float weight[8]={6,3.5,2,1.8,-1.8,-2,-3.5,-6};

#define KP 0.3 // 比例系数
#define KI 0     // 积分系数
#define KD 0.62  // 微分系数
void Beep_on()
{
	
		DL_GPIO_clearPins(Beep_PORT,Beep_PIN_2_PIN);
delay_ms(200);
	DL_GPIO_setPins(Beep_PORT,Beep_PIN_2_PIN);


}
void Auto_Calibrate(No_MCU_Sensor *sensor_ptr)
{
Beep_on();
//	uart0_send_string("put it to white...\r\n");
		while(1)
		{
		if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0)
		{
		delay_ms(20);
		while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0);
		delay_ms(20);

				break;
		}
		}
		delay_ms(50);
		unsigned short white[8];
		unsigned short black[8];
		No_MCU_Ganv_Sensor_Init_Frist(sensor_ptr);
		No_Mcu_Ganv_Sensor_Task_Without_tick(sensor_ptr);
		Get_Anolog_Value(&sensor,Anolog);

//		sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
//		uart0_send_string((char *)rx_buff);

		// 采样白色值
		for (int j = 0; j < 10; j++)
		{
			No_MCU_Ganv_Sensor_Init_Frist(sensor_ptr);
			No_Mcu_Ganv_Sensor_Task_Without_tick(sensor_ptr);
			Get_Anolog_Value(sensor_ptr,Anolog);
			for (int i = 0; i < 8; i++)
				white[i] = Anolog[i];
			delay_ms(20);
		}
		Beep_on();

	///	uart0_send_string("put it to black...\r\n");
		while(1)
		{
		if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0)
		{
		delay_ms(20);
		while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0);
		delay_ms(20);
								break;

		}

		}
		delay_ms(50);
		// 采样黑色值
		for (int j = 0; j < 10; j++)
		{
			No_MCU_Ganv_Sensor_Init_Frist(sensor_ptr);
			No_Mcu_Ganv_Sensor_Task_Without_tick(sensor_ptr);
			Get_Anolog_Value(sensor_ptr,Anolog);
			for (int i = 0; i < 8; i++)
				black[i] = Anolog[i];  
//			uart0_send_string("ok...\r\n");

			delay_ms(20);
		}
		//    for (int i = 0; i < 8; i++)
		//        black[i] /= 10;
			No_MCU_Ganv_Sensor_Init(sensor_ptr,white,black);
//		uart0_send_string("okq...\r\n");
//		uart0_send_string("okq...\r\n");
Beep_on();


}


int main(void)
{
	SYSCFG_DL_init();
	board_init();

//	Auto_Calibrate(&sensor);
	No_MCU_Sensor sensor;
	unsigned char Digtal;
SYSCFG_DL_init();
	//    NVIC_EnableIRQ(ENCODERL_INT_IRQN);//开启按键引脚的GPIOB端口中断

		DL_TimerA_startCounter(PWM_INST);//开始计数
	float integral_error = 0; // 积分误差
	float last_error = 0;     // 上一次的误差，用于计算微分项
	int keynum =Count_Key1_Press();

	
	int rightangle=0;
if(keynum>=1)
{
	Auto_Calibrate(&sensor);
}
else
{
	No_MCU_Ganv_Sensor_Init_Frist(&sensor);
	No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
//	Get_Anolog_Value(&sensor,Anolog);
	Get_Anolog_Value(&sensor,Anolog);
//	sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
//			uart0_send_string((char *)rx_buff);
	No_MCU_Ganv_Sensor_Init(&sensor,white,black);
	delay_ms(100);

}
 int roundnum =Count_Key1_Press();

int jiao_turn = 0;

	//TIMG0_Init();

while (1) 
	

{ 
//				sprintf((char *)rx_buff,"Ex_cnt_right %d\r\n",Ex_cnt_right);
//			uart0_send_string((char *)rx_buff);
	
	No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
	if(Get_Normalize_For_User(&sensor,Normal))
	{
		int error=0 ;
		int  blackvalue=0;
		int  sumblackvalue=0;
////************************************************************************************************//
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
//		sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
//		uart0_send_string((char *)rx_buff);
//		memset(rx_buff,0,256);
////************************************************************************************************//
				
//***************************丢线判断*********************************************************************//
			

		if((Normal[0]>3000&&Normal[1]>3000&&Normal[2]>3000&&Normal[3]>3000&&Normal[4]>3000&&Normal[5]>3000&&Normal[6]>3000&&Normal[7]>3000))
		{		
//			uart0_send_char((char)5);
//			uart0_send_string("Tuo");
			jiao_turn++;
			delay_ms(50);
											rightangle++;
			uart0_send_char(rightangle);
//			sprintf((char *)rx_buff,"rightangle %d\r\n",rightangle);
//			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);

				AO_Control(1,0);
				BO_Control(1,800);

				delay_ms(200);
						if(rightangle==4*roundnum)
			{
				Stop();
				while(1);
			
			}

			while((Normal[0]>3000&&Normal[1]>3000&&Normal[2]>3000&&Normal[3]>3000&&Normal[4]>3000&&Normal[5]>3000&&Normal[6]>3000&&Normal[7]>3000))
			{
				No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
				Get_Normalize_For_User(&sensor,Normal);
				delay_ms(10);
			}

		
		}
//				
		else
		{	
////************************************************************************************************//
//***************************pid*********************************************************************//
			delay_ms(1);
			for (int i = 0; i < 8; i++) 
			{
				blackvalue=4096-Normal[i];
				sumblackvalue=sumblackvalue+blackvalue;
				error=error+blackvalue* weight[i];
			}
			
			if(sumblackvalue!=0)
			error=100*error/sumblackvalue;
			
			
//************************************************************************************************//


			float p_term = KP * error;
			integral_error += error;
			if (integral_error > 300) integral_error = 300;
			if (integral_error < -300) integral_error = -300;
			float i_term = KI * integral_error;
			float derivative_error = error - last_error;
			float d_term = KD * derivative_error;
			int steering_correction = p_term + i_term + d_term;

		if (my_abs(steering_correction) < 1)  // 允许一个微小误差
			steering_correction = 0;
		Set_Motor_Speeds(-steering_correction);


last_error = error;
	}
		delay_ms(1);
	 }
 }
	}

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//			{
//			sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
//			uart0_send_string((char *)rx_buff);
//			memset(rx_buff,0,256);
//			}
//						//获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
//			if(Get_Anolog_Value(&sensor,Anolog)){
//			sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
//			uart0_send_string((char *)rx_buff);
//			memset(rx_buff,0,256);
//			}
	 
	 
 //			if(Get_Normalize_For_User(&sensor,Normal))
//				{
//			sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
//			uart0_send_string((char *)rx_buff);
//			memset(rx_buff,0,256);
//			}
			
//						Digtal=Get_Digtal_For_User(&sensor);
//			sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
//			uart0_send_string((char *)rx_buff);
//			memset(rx_buff,0,256);
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)//b0 wDA