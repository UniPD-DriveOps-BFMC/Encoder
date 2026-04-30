#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update(void);

int16_t Encoder_GetDelta(void);
int32_t Encoder_GetTotal(void);
uint16_t Encoder_GetRaw(void);

#endif
