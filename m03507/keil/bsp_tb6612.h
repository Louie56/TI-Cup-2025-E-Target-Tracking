#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include "board.h"
#include "ti_msp_dl_config.h"


#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(MOTOR_A_PORT,MOTOR_A_AIN1_PIN)) : (DL_GPIO_clearPins(MOTOR_A_PORT,MOTOR_A_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(MOTOR_A_PORT,MOTOR_A_AIN2_PIN)) : (DL_GPIO_clearPins(MOTOR_A_PORT,MOTOR_A_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(MOTOR_B_PORT,MOTOR_B_BIN1_PIN)) : (DL_GPIO_clearPins(MOTOR_B_PORT,MOTOR_B_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(MOTOR_B_PORT,MOTOR_B_BIN2_PIN)) : (DL_GPIO_clearPins(MOTOR_B_PORT,MOTOR_B_BIN2_PIN)) )

void AIN_GPIO_INIT(void);
void BIN_GPIO_INIT(void);
void TB6612_Init(uint16_t pre,uint16_t per);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);


#endif  /* _BSP_TB6612_H */
 