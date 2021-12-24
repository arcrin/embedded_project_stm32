//
// Created by wbai on 12/23/2021.
//

#include "stm32f407xx_gpio_driver.h"
#include <cstdint>


void delay(){
    for (int i = 0; i < 500000; i++);
}

int main() {
    GPIO_Handle_t led_gpio_handle;
    led_gpio_handle.pGPIOx = GPIOD;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    led_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // do not use open drain unless there is a specific reason

    led_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&led_gpio_handle);
    GPIO_PeriClockControl(led_gpio_handle.pGPIOx, ENABLE);

    while (1) {
        // the led will toggle with very low intensity, the internal pull-up resistor is too high
        delay();
        GPIO_ToggleOutputPin(led_gpio_handle.pGPIOx, GPIO_PIN_NO_12);
    }

}