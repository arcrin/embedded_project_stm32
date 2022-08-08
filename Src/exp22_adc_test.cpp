//
// Created by andy- on 2022-08-06.
//
#include "stm32f407xx.h"


uint32_t adc1_pa1_val;


int main(){
    disable_irq();
    SysTick_Init(16000);
    ADC_Init();
    Start_Conversion();
    enable_irq();
    adc1_pa1_val = ADC_Read();
    while (1){
//        delay(1000);
//        adc1_pa1_val = ADC_Read();
    }
}