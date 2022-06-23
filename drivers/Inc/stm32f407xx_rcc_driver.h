//
// Created by wbai on 6/17/2022.
//

#ifndef MCU1_STM32_RCC_DRIVER_H
#define MCU1_STM32_RCC_DRIVER_H
#include "stm32f407xx.h"

#define RCC_CLK_SRC_HSI     0
#define RCC_CLK_SRC_HSE     1
#define RCC_CLK_SRC_PLL     2

// Set clock source
void RCC_SetClockSource(uint8_t clock_source);

// APB1 clock value
uint32_t RCC_GetPCLK1Value();

// APB2 clock value
uint32_t RCC_GetPCLK2Value();

uint32_t RCC_GetPLLOutputClock();

#endif //MCU1_STM32_RCC_DRIVER_H
