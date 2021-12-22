//
// Created by andy- on 2021-10-26.
//

#include "stm32f407xx_gpio_driver.h"
#include <cstdint>


int main() {
    GPIO_Handle_t gpio_handle;
    gpio_handle.pGPIOx = GPIOD;
    gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // pin output type is already push-pull, no need for pu/pd resistors

    GPIO_PeriClockControl(gpio_handle.pGPIOx, ENABLE);
    GPIO_Init(&gpio_handle);
//    GPIO_ToggleOutputPin(gpio_handle->pGPIOx, GPIO_PIN_NO_12);
    GPIO_WriteOutputPin(gpio_handle.pGPIOx, GPIO_PIN_NO_12, GPIO_PIN_RESET);
}