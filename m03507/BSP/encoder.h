#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include "board.h"
void encoder_init(void);
extern int encoder_val;
extern int encoder_val2;

int32_t Get_Encoder1_Count(void);
int32_t Get_Encoder2_Count(void);
#endif
