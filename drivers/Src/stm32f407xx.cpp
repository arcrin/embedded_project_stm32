//
// Created by wbai on 5/10/2022.
//
#include "stm32f407xx.h"
#include <cstdint>

_vo uint32_t uwTick;

HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;

void TIM4_IRQHandler(){

}

uint32_t Get_Tick(){
    return uwTick;
}

void HAL_Delay(uint32_t Delay){
    uint32_t tickstart = Get_Tick();
    uint32_t wait = Delay;

    // Add a freq to guarantee minimum wait
    if(wait < HAL_MAX_DELAY) {
        wait += (uint32_t) (uwTickFreq);
    }

    while ((Get_Tick() - tickstart) < wait);
}