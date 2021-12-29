//
// Created by andy- on 2021-10-26.
//

#include "stm32f407xx_gpio_driver.h"
#include <cstdint>

void delay(){
    for (uint32_t i = 0; i < 500000; i++);
}

int main() {
    GPIO_Handle_t led_gpio_handle;
    led_gpio_handle.pGPIOx = GPIOD;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // pin output type is already push-pull, no need for pu/pd resistors

    GPIO_PeriClockControl(led_gpio_handle.pGPIOx, ENABLE);
    GPIO_Init(&led_gpio_handle);

    while(1){
        GPIO_ToggleOutputPin(led_gpio_handle.pGPIOx, led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber);
        delay();
    }
}