//
// Created by wbai on 12/23/2021.
//

#include "stm32f407xx_gpio_driver.h"
#include <cstdint>


//void delay(){
//    for (int i = 0; i < 500000; i++);
//}

int main() {
    disable_irq();
    NVIC_SetPriority(-1, 0);
    GPIO_Handle_t led_gpio_handle;
    led_gpio_handle.pGPIOx = GPIOD;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    //TODO: Why doesn't open drain work?
    led_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // do not use open drain unless there is a specific reason
    led_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


    GPIO_Init(&led_gpio_handle);

    SysTick_Init(16000);

    enable_irq();
    while (1) {
        delay(1000);
        GPIO_ToggleOutputPin(led_gpio_handle.pGPIOx, GPIO_PIN_NO_12);
    }
}