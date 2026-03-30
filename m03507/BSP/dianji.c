#include "stdio.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "dianji.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "ti_msp_dl_config.h"

#define speedA 690//640
#define speedB 690
#define basespeed 1000
//1.616比率
int speedsetA,speedsetB;
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
	if(speed>3000)
		return 3000;
	if(speed<300)
		return 300;//300
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



//unsigned short Normal[8];
//unsigned char rx_buff[256]={0};

//void get_track_values()
//{
//	unsigned char Digtal;
//		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
//	//有时基传感器常规任务，包含模拟量，数字量，归一化量
////			No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
//	//获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
//	Digtal=Get_Digtal_For_User(&sensor);
////	sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
////	uart0_send_string((char *)rx_buff);
//	memset(rx_buff,0,256);
//}


//double integral_error = 0; // 积分误差
//double last_error = 0;     // 上一次的误差，用于计算微分项

//#define KP 0.5   // 比例系数 (Proportional)
//#define KI 0.001 // 积分系数 (Integral) - 注意：积分项在循迹中可能需要谨慎使用，容易引起震荡
//#define KD 0.1   // 微分系数 (Derivative)
void Set_Motor_Speeds(int steering_correction)
{	

	v_left=max(basespeed+steering_correction);
	v_right=max(basespeed-steering_correction);
//	v_left=1000;//hou
//	v_right=1000;//q
	if (v_left<0)
		AO_Control(-1,-v_left);//h
	if(v_left>0)
		AO_Control(+1,v_left);//q
	delay_ms(50);

	if(v_right<0)
	BO_Control(-1,-v_right);//h
	if(v_right>0)
	BO_Control(+1,+v_right);//q
	delay_ms(50);

}	






//void Run_PID_LineFollowing() {
//    unsigned short normalized_values[8];
//	unsigned short Anolog[8]={0};
//	unsigned char rx_buff[256]={0};


//    if (Get_Normalize_For_User(&sensor, normalized_values)) 
//		{
//			long position_error_sum = 0;
//			int active_sensors_sum = 0; // 用于计算加权平均分母，避免除以零
//			// 传感器0在最左边，传感器7在最右边
//			// 黑线值低，白线值高。我们希望检测黑线，所以用 (mySensor.bits - normalized_values[i])
//			// 这样黑线上的传感器贡献的权重会更高

//	//加权平均
//			for (int i = 0; i < 8; i++) 
//			{
//				// 计算每个传感器对位置的贡献，并累加活跃度（即传感器亮度反转值）
//				position_error_sum += (long)i * (sensor.bits - normalized_values[i]);
//				active_sensors_sum += (sensor.bits - normalized_values[i]);
//			}
//			

//			double current_line_position = 0;
//			
//			// 避免除以零
//			if (active_sensors_sum > 0) {
//				current_line_position = (double)position_error_sum / active_sensors_sum;
//			} else {
//				// 如果所有传感器都在白色区域（active_sensors_sum为0），
//				// 这可能意味着小车脱离了黑线。
//				// 此时可以根据情况进行特殊处理，例如停止或原地旋转寻找线。
//				// 这里我们简单地保持上一刻的误差，或者设置为一个默认值。
//				current_line_position = last_error; // 或者设置为中心值，取决于脱线策略
//			}

//			// 3. 计算误差
//			// 理想的中心位置。对于8个传感器（0-7），中心在 (0+7)/2 = 3.5
//			double ideal_center = 3.5;
//			double error = current_line_position - ideal_center;//左偏error为负

//			// 4. 计算 PID 控制项
//			// 比例项 P
//			double p_term = KP * error;

//			// 积分项 I (可选，但要小心使用，容易导致过冲)
//			// 积分项的累加通常在误差接近零时才有效，防止在远离目标时积分累积过大 (积分饱和)
//			// 可以加入积分限幅
//			integral_error += error;
//			// 积分限幅示例
//			if (integral_error > 300) integral_error = 300;
//			if (integral_error < -300) integral_error = -300;
//			double i_term = KI * integral_error;

//			// 微分项 D
//			double derivative_error = error - last_error;
//			double d_term = KD * derivative_error;
//			// 5. 计算总的转向修正量
//			double steering_correction = p_term + i_term + d_term;

//	//        // 6. 限幅转向修正量，防止过大的转弯导致小车失控
//	//        if (steering_correction > MAX_STEERING_CORRECTION) {
//	//            steering_correction = MAX_STEERING_CORRECTION;
//	//        } else if (steering_correction < -MAX_STEERING_CORRECTION) {
//	//            steering_correction = -MAX_STEERING_CORRECTION;
//	//        }

//			// 8. 调用实际的电机控制函数来设置速度
//			Set_Motor_Speeds(steering_correction);
//			// 9. 更新上一次误差
//			last_error = error;
//	}
////	else 
////	{
////		Stop();
////    }
//}







