//
// Created by wbai on 12/23/2021.
//

//
// Created by wbai on 12/23/2021.
//

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <cstdint>
#include <cstring>


void manual_delay(uint8_t scale){
    for (int i = 0; i < 500000/scale; i++);
}

int main() {
    disable_irq();
    NVIC_SetPriority(-1, 0);

    GPIO_Handle_t ld4_gpio_handle, b1_gpio_handle;
    memset(&ld4_gpio_handle, 0, sizeof(ld4_gpio_handle));
    memset(&b1_gpio_handle, 0, sizeof(b1_gpio_handle));
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
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_RT_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&b1_gpio_handle);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
    GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);

    SysTick_Init(16000);
    enable_irq();

    while(1){
//        delay(5);
//        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    }
}

extern "C"{ // use extern "C" to prevent C++ name mangling
    void EXTI0_IRQHandler() {
//        delay(10); //TODO: seems like there is debouncing problem
        GPIO_IRQHandling(0);
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    }
}
