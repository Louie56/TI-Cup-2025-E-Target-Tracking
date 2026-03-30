#include "encoder.h"

/*E1A PA27  E1B PA26
  E2A PA25  E2B PA24*/
	
//1圈728 13*28*4 /2 不知道2哪来的
//右A左B

int encoder_val = 0;
int encoder_val2 = 0;
int speed = 0;
int speed2 = 0;
int last_encoder_val = 0;
int last_encoder_val2 = 0;


int cnt_encoder = 0;


void encoder_init(void)
{
	NVIC_EnableIRQ(GPIOA_INT_IRQn);
}


#define Decoder_PORT GPIOA
#define Decoder_Decoder_A_IIDX  DL_GPIO_PIN_TO_IIDX(Decoder_PORT, Decoder_Decoder_A_PIN)

int32_t Ex_cnt_right = 0;
int32_t Ex_cnt_left = 0;


#define DECODER_A_PIN_MASK  DL_GPIO_PIN_27
#define DECODER_B_PIN_MASK  DL_GPIO_PIN_26

#define DECODER_A2_PIN_MASK DL_GPIO_PIN_25
#define DECODER_B2_PIN_MASK DL_GPIO_PIN_24
////右
//void Soft_Decoder(void)
//{
//    // 1. 获取所有启用并挂起的中断状态
//    uint32_t enabled_pending = DL_GPIO_getEnabledInterruptStatus(Decoder_PORT, 
//                                  DECODER_A_PIN_MASK | DECODER_B_PIN_MASK);
//    
//    // 2. 判断中断源 (更高效的方法)
//    uint8_t A = (enabled_pending & DECODER_A_PIN_MASK) ? 1 : 0;
//    uint8_t B = (enabled_pending & DECODER_B_PIN_MASK) ? 1 : 0;
//    
//    // 3. 计算引脚变化 (与原始逻辑一致)
//    uint8_t pinA_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_A_PIN_MASK);
//    uint8_t pinB_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_B_PIN_MASK);
//    
//    uint8_t Temp = A + pinA_state + pinB_state;
//    
//    // 4. 判断旋转方向
//    if (Temp % 2) 
//        Ex_cnt_right--;  // 奇数减少计数
//    else 
//        Ex_cnt_right++;  // 偶数增加计数
//    
//    // 5. 清除中断标志
//    if (A) DL_GPIO_clearInterruptStatus(Decoder_PORT, DECODER_A_PIN_MASK);
//    if (B) DL_GPIO_clearInterruptStatus(Decoder_PORT, DECODER_B_PIN_MASK);
//}



//void Soft_Decoder2(void)
//{
//    // 1. 获取所有启用并挂起的中断状态
//    uint32_t enabled_pending = DL_GPIO_getEnabledInterruptStatus(Decoder_PORT, 
//                                  DECODER_A2_PIN_MASK | DECODER_B2_PIN_MASK);
//    
//    // 2. 判断中断源 (更高效的方法)
//    uint8_t A = (enabled_pending & DECODER_A2_PIN_MASK) ? 1 : 0;
//    uint8_t B = (enabled_pending & DECODER_B2_PIN_MASK) ? 1 : 0;
//    
//    // 3. 计算引脚变化 (与原始逻辑一致)
//    uint8_t pinA_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_A2_PIN_MASK);
//    uint8_t pinB_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_B2_PIN_MASK);
//    
//    uint8_t Temp = A + pinA_state + pinB_state;
//    
//    // 4. 判断旋转方向
//    if (Temp % 2) 
//        Ex_cnt_left--;  // 奇数减少计数
//    else 
//        Ex_cnt_left++;  // 偶数增加计数
//    
//    // 5. 清除中断标志
//    if (A) DL_GPIO_clearInterruptStatus(Decoder_PORT, DECODER_A2_PIN_MASK);
//    if (B) DL_GPIO_clearInterruptStatus(Decoder_PORT, DECODER_B2_PIN_MASK);
//}

//// 正交编码器中断服务函数
//// 修改后的中断服务函数
//void GROUP1_IRQHandler(void)
//{
//    // 1. 获取所有待处理的中断标志
//    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(Decoder_PORT, 
//        DECODER_A_PIN_MASK | DECODER_B_PIN_MASK | 
//        DECODER_A2_PIN_MASK | DECODER_B2_PIN_MASK);
//    
//    // 2. 只处理实际触发的中断
//    if (pending & (DECODER_A_PIN_MASK | DECODER_B_PIN_MASK)) {
//        Soft_Decoder();  // 只处理第一个编码器
//    }
//    
//    if (pending & (DECODER_A2_PIN_MASK | DECODER_B2_PIN_MASK)) {
//        Soft_Decoder2(); // 只处理第二个编码器
//    }
//}

// 右轮编码器解码
void Soft_Decoder(uint32_t pending)
{
    // 1. 判断中断源
    uint8_t A = (pending & DECODER_A_PIN_MASK) ? 1 : 0;
    uint8_t B = (pending & DECODER_B_PIN_MASK) ? 1 : 0;
    
    // 2. 读取引脚状态
    uint8_t pinA_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_A_PIN_MASK);
    uint8_t pinB_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_B_PIN_MASK);
    
    // 3. 计算引脚变化
    uint8_t Temp = A + pinA_state + pinB_state;
    
    // 4. 判断旋转方向
    if (Temp % 2) 
        Ex_cnt_right--;  // 奇数减少计数
    else 
        Ex_cnt_right++;  // 偶数增加计数
}

// 左轮编码器解码
void Soft_Decoder2(uint32_t pending)
{
    // 1. 判断中断源
    uint8_t A = (pending & DECODER_A2_PIN_MASK) ? 1 : 0;
    uint8_t B = (pending & DECODER_B2_PIN_MASK) ? 1 : 0;
    
    // 2. 读取引脚状态
    uint8_t pinA_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_A2_PIN_MASK);
    uint8_t pinB_state = !!DL_GPIO_readPins(Decoder_PORT, DECODER_B2_PIN_MASK);
    
    // 3. 计算引脚变化
    uint8_t Temp = A + pinA_state + pinB_state;
    
    // 4. 判断旋转方向
    if (Temp % 2) 
        Ex_cnt_left--;  // 奇数减少计数
    else 
        Ex_cnt_left++;  // 偶数增加计数
}
// 防卡死中断服务函数（无看门狗）
void GROUP1_IRQHandler(void)
{
    // 1. 进入临界区（禁用全局中断）
    __disable_irq();
    
    // 2. 获取所有待处理的中断标志
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(Decoder_PORT, 
        DECODER_A_PIN_MASK | DECODER_B_PIN_MASK | 
        DECODER_A2_PIN_MASK | DECODER_B2_PIN_MASK);
    
    // 3. 保存中断状态用于后续处理
    uint32_t pending1 = pending & (DECODER_A_PIN_MASK | DECODER_B_PIN_MASK);
    uint32_t pending2 = pending & (DECODER_A2_PIN_MASK | DECODER_B2_PIN_MASK);
    
    // 4. 立即清除所有中断标志（防止重入）
    DL_GPIO_clearInterruptStatus(Decoder_PORT, pending);
    
    // 5. 退出临界区（允许新中断）
    __enable_irq();
    
    // 6. 处理中断（在中断启用状态下）
    if (pending1) {
        Soft_Decoder(pending1);
    }
    
    if (pending2) {
        Soft_Decoder2(pending2);
    }
}