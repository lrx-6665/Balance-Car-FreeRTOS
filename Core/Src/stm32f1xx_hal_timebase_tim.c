/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_hal_timebase_tim.c
  * @brief   HAL time base：TIM1（电机 PWM 占用 TIM2，编码器占用 TIM3/TIM4）
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

TIM_HandleTypeDef htim1;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock, uwAPB2Prescaler = 0U;
  uint32_t uwPrescalerValue = 0U;
  uint32_t pFLatency;
  HAL_StatusTypeDef status = HAL_OK;

  /* TIM1 在 APB2 */
  __HAL_RCC_TIM1_CLK_ENABLE();

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwAPB2Prescaler = clkconfig.APB2CLKDivider;
  if (uwAPB2Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK2Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK2Freq();
  }

  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

  htim1.Instance = TIM1;
  htim1.Init.Period = (1000000U / 1000U) - 1U;
  htim1.Init.Prescaler = uwPrescalerValue;
  htim1.Init.ClockDivision = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim1);
  if (status == HAL_OK)
  {
    status = HAL_TIM_Base_Start_IT(&htim1);
    if (status == HAL_OK)
    {
      /* F103：TIM1 更新中断是 TIM1_UP_IRQn */
      HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        HAL_NVIC_SetPriority(TIM1_UP_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

  return status;
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}
