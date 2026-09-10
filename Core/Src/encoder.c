#include "encoder.h"
#include "tim.h"

/* 江协：左编码器 TIM3(PA6/PA7)，右编码器 TIM4(PB6/PB7) */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

int16_t Encoder_ReadLeft(void)
{
    int16_t v = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return v;
}

int16_t Encoder_ReadRight(void)
{
    int16_t v = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return v;
}
