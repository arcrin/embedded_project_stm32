//
// Created by wbai on 6/17/2022.
//

#ifndef MCU1_STM32_RCC_DRIVER_H
#define MCU1_STM32_RCC_DRIVER_H
#include "stm32f407xx.h"

// APB1 clock value
uint32_t RCC_GetPCLK1Value();

// APB2 clock value
uint32_t RCC_GetPCLK2Value();

uint32_t RCC_GetPLLOutputClock();

#endif //MCU1_STM32_RCC_DRIVER_H
