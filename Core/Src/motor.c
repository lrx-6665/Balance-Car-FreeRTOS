#include "motor.h"
#include "main.h"
#include "tim.h"

/* 江协：PWM = TIM2 CH1/CH2 → PA0/PA1；方向 PB12~15 */
extern TIM_HandleTypeDef htim2;

#define PWM_MAX 100

void Motor_Init(void)
{
    /* PA15 = TB6612 STBY，高电平使能电机 */
    HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Motor_SetSpeed(int16_t left, int16_t right)
{
    int16_t pl = left;
    int16_t pr = right;

    if (pl > PWM_MAX) pl = PWM_MAX;
    if (pl < -PWM_MAX) pl = -PWM_MAX;
    if (pr > PWM_MAX) pr = PWM_MAX;
    if (pr < -PWM_MAX) pr = -PWM_MAX;

    /* 左电机：正转 PB12=1 PB13=0（与江协一致） */
    if (pl >= 0) {
        HAL_GPIO_WritePin(M1_A_GPIO_Port, M1_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_B_GPIO_Port, M1_B_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)pl);
    } else {
        HAL_GPIO_WritePin(M1_A_GPIO_Port, M1_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_B_GPIO_Port, M1_B_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(-pl));
    }

    /* 右电机：正转 PB14=0 PB15=1（江协与左轮方向脚逻辑相反） */
    if (pr >= 0) {
        HAL_GPIO_WritePin(M2_A_GPIO_Port, M2_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2_B_GPIO_Port, M2_B_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)pr);
    } else {
        HAL_GPIO_WritePin(M2_A_GPIO_Port, M2_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2_B_GPIO_Port, M2_B_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(-pr));
    }
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}
