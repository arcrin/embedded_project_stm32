//
// Created by wbai on 12/23/2021.
//

//
// Created by wbai on 12/23/2021.
//

#include "stm32f407xx_gpio_driver.h"
#include <cstdint>


void delay(){
    for (int i = 0; i < 500000/10; i++);
}

int main() {
    GPIO_Handle_t ld4_gpio_handle, b1_gpio_handle;
    ld4_gpio_handle.pGPIOx = GPIOD;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOD, ENABLE); // this needs to be called before GPIO_Init
    GPIO_Init(&ld4_gpio_handle);

    b1_gpio_handle.pGPIOx = GPIOA;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&b1_gpio_handle);

    while (1) {
        if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == ENABLE) {
            delay();
            GPIO_WriteOutputPin(ld4_gpio_handle.pGPIOx, ld4_gpio_handle.GPIO_PinConfig.GPIO_PinNumber, ENABLE);
        } else
        {
            delay();
            GPIO_WriteOutputPin(ld4_gpio_handle.pGPIOx, ld4_gpio_handle.GPIO_PinConfig.GPIO_PinNumber, DISABLE);
        }
    }
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){}