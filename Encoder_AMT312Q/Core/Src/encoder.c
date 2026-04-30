#include "encoder.h"

static TIM_HandleTypeDef *encoder_timer = 0;

volatile int16_t encoder_delta = 0;
volatile int32_t encoder_total = 0;
volatile uint16_t encoder_raw = 0;

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    encoder_timer = htim;

    HAL_TIM_Encoder_Start(encoder_timer, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(encoder_timer, 0);

    encoder_delta = 0;
    encoder_total = 0;
    encoder_raw = 0;
}

void Encoder_Update(void)
{
    encoder_raw = __HAL_TIM_GET_COUNTER(encoder_timer);

    encoder_delta = (int16_t)encoder_raw;

    encoder_total += encoder_delta;

    __HAL_TIM_SET_COUNTER(encoder_timer, 0);
}

int16_t Encoder_GetDelta(void)
{
    return encoder_delta;
}

int32_t Encoder_GetTotal(void)
{
    return encoder_total;
}

uint16_t Encoder_GetRaw(void)
{
    return encoder_raw;
}
