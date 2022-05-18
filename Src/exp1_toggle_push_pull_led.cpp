//
// Created by andy- on 2021-10-26.
//

#include "stm32f407xx.h"

#include <cstdint>

GPIO_Handle_t led_gpio_handle; // pGPIO_Handle_t wouldn't work here. It will be initialized as a NULL pointer, and content will be corrupted
GPIO_Handle_t pd2_gpio_handle;
int main() {
    disable_irq();
    NVIC_SetPriority(-1, 0);

    led_gpio_handle.pGPIOx = GPIOD;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // pin output type is already push-pull, no need for pu/pd resistors

//    pd2_gpio_handle.pGPIOx = GPIOD;
//    pd2_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
//    pd2_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
//    pd2_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//    pd2_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//    pd2_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    SysTick_Init(16000);

    GPIO_Init(&led_gpio_handle);
    GPIO_Init(&pd2_gpio_handle);

    enable_irq();

    while(1){
//        GPIO_ToggleOutputPin(pd2_gpio_handle.pGPIOx, pd2_gpio_handle.GPIO_PinConfig.GPIO_PinNumber);
        GPIO_ToggleOutputPin(led_gpio_handle.pGPIOx, led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber);
        delay(1000);
    }
}





