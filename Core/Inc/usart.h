/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void);
void BleUart_RestartRx(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
