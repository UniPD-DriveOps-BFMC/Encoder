#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

#define ENCODER_CPR          8192
#define GEAR_NUM             66
#define GEAR_DEN             527
#define WHEEL_RADIUS_M       0.03224f
#define ENCODER_TS_S         0.01f
#define VELOCITY_LPF_ALPHA   0.10f
#define Z_CORRECTION_GAIN    0.20f

typedef struct
{
    int16_t delta_counts;
    int32_t total_counts_raw;
    int32_t total_counts_corrected;

    float distance_m;
    float velocity_mps;
    float velocity_mps_filtered;
    float acceleration_mps2;

    uint8_t z_seen;
    uint32_t z_count;
} Encoder_State_t;

void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update_10ms_ISR(void);
void Encoder_Z_ISR(void);
Encoder_State_t Encoder_GetState(void);

#endif
