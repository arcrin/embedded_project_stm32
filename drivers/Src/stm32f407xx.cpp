//
// Created by wbai on 5/10/2022.
//
#include "stm32f407xx.h"
#include <cstdint>

_vo uint32_t sysTick;

void NVIC_SetPriority(int8_t IRQn, uint8_t priority){
    if (IRQn >= 0){
        NVIC->IP[IRQn] = (priority << (uint8_t)((8 - NVIC_PRIORITY_BITS)) & (uint32_t)0xFFUL);
    }
    else
    {
        SCB->SHPR[(IRQn & 0xF) - 4] = (priority << (uint8_t)((8 - NVIC_PRIORITY_BITS)) & (uint32_t)0xFFUL);
    }
}

uint32_t get_tick(){
    return sysTick;
}

void delay(uint32_t delay_in_ms){
    uint32_t start_tick = get_tick();
    uint32_t wait = delay_in_ms;
    // TODO: what does this code do? Guarantee minimum wait?
//    if (delay_in_ms < 0xFFFFFFFFU){
//        wait += (uint32_t) (0x1);
//    }
    while ((get_tick() - start_tick) < wait){
    }
}

extern "C"{
    void SysTick_Handler(){
        sysTick++;
    }
}