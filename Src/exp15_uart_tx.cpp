//
// Created by wbai on 6/20/2022.
//
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void USART2_Init(){
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_TX_ONLY_MODE;
    usart2_handle.USART_Config.USART_NumOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

void USART2_GPIOInit(){
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    // USART2 TX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    // USART2 RX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

void GPIO_ButtonInit(){
    GPIO_Handle_t GPIOButton, GPIOLed;

    // this is button gpio configuration
    GPIOButton.pGPIOx = GPIOA;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOButton);

    // this is led gpio configuration
    GPIOLed.pGPIOx = GPIOD;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&GPIOLed);
}

int main(){
    disable_irq();
    RCC_SetClockSource(RCC_CLK_SRC_HSI);
    GPIO_ButtonInit();
    USART2_GPIOInit();

    USART2_Init();
    USART_PeripheralControl(USART2, ENABLE);
    SysTick_Init(16000);
    enable_irq();
    while (1) {
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(200);
        USART_SendData(&usart2_handle, (uint8_t *) msg, strlen(msg));
    }
    return 0;
}