#ifndef __ENCODER_H
#define __ENCODER_H
#include "tim.h"

void Encoder_Init(void);
int16_t Encoder_Read_CNT(uint8_t Hx);
void Encoder_ShowCnt(void);

#endif //ROBOT_ENCODER_H
