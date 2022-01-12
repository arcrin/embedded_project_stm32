//
// Created by wbai on 12/23/2021.
//

//
// Created by wbai on 12/23/2021.
//
#include "stm32f407xx_gpio_driver.h"
#include <cstdint>

#define PRESSED     ENABLE
#define RELEASED    DISABLE

void delay(){
    for (int i = 0; i < 500000/2; i++);
}

int main() {
    GPIO_Handle_t ext_led_gpio_handle, ext_btn_gpio_handle;
    ext_led_gpio_handle.pGPIOx = GPIOA;
    ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(ext_led_gpio_handle.pGPIOx, ENABLE); // this needs to be called before GPIO_Init
    GPIO_Init(&ext_led_gpio_handle);

    ext_btn_gpio_handle.pGPIOx = GPIOB;
    ext_btn_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    ext_btn_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    ext_btn_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    ext_btn_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(ext_btn_gpio_handle.pGPIOx, ENABLE);
    GPIO_Init(&ext_btn_gpio_handle);
    while(1){
        if (GPIO_ReadFromInputPin(ext_btn_gpio_handle.pGPIOx, ext_btn_gpio_handle.GPIO_PinConfig.GPIO_PinNumber) ==
            PRESSED) {
            GPIO_WriteOutputPin(ext_led_gpio_handle.pGPIOx, ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber, DISABLE);
        } else {
            GPIO_WriteOutputPin(ext_led_gpio_handle.pGPIOx, ext_led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber, ENABLE);
        }
    }
}