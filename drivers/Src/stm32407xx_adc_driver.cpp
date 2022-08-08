//
// Created by andy- on 2022-08-06.
//
#include "stm32f407xx.h"

void  ADC_Init(){
    // Enable clock to ADC pin PORT PA1
    RCC->AHB1ENR |= (1 << 0);

    // Set mode of PA1 to analog
    GPIOA->MODER |= (3 << 2 * 1);

    // Configure ADC peripheral
    // enable clock to ADC
    RCC->APB2ENR |= (1 << 8);

    // configure ADC parameters
    // conversion SEQ start
    ADC1->SQR3 |= (1 << 5 * 0);

    // conversion SEQ length
    ADC1->SQR1 |= (0 << 20);

    // enable ADC
    ADC1->CR2 = (1 << 0);
}

void Start_Conversion(){
    // Start ADC conversion
    ADC1->CR2 |= (1 << 30); // Bit 30 SWSTART: Start conversion of regular channels
}

uint32_t ADC_Read(){
    // Wait for conversion to complete
    // ADC_SR: Bit 1 EOC: Regular channel end of conversion
    while((ADC1->SR & (1 << 1)) == 0);

    // read converted result
    // ADC_DR: Bits 15:0 DATA[15:0]: Regular data; default: right aligned i.e. 12 bit from right in regular mode
    uint32_t value = (ADC1->DR);
    return value;
}



