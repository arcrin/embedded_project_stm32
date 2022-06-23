//
// Created by wbai on 6/21/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"

// we have 3 different messages that we transmit to arduino
char *msg[3] = {(char*)"hihihihihihi123\n", (char*)"Hello, how are you?\n", (char*)"Today is Monday!\n"};

// reply from arduino will be stored here
char rx_buf[1024];

USART_Handle_t usart2_handle;

// This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;

void USART2_Init(){
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_TXRX_MODE;
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

    // USART2 Tx pin
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    // USART2 Rx pin
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}


void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOButton,GpioLed;

    //this is btn gpio configuration
    GPIOButton.pGPIOx = GPIOA;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOButton);

    //this is led gpio configuration
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD,ENABLE);

    GPIO_Init(&GpioLed);
}

int main() {
    disable_irq();
    uint32_t count = 0;
    RCC_SetClockSource(RCC_CLK_SRC_HSI);
    SysTick_Init(16000);
    USART2_GPIOInit();
    USART2_Init();
    GPIO_ButtonInit();
    USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
    USART_PeripheralControl(USART2, ENABLE);
    enable_irq();

    while(1) {
        // wait for the button press
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
            // to avoid button de-bouncing related issues, 200 ms delay
        delay(200);

        // Next message index, make sure that count value doesn't cross 2
        count = count % 3;

        // First lets enable the reception in interrupt mode
        while (USART_ReceiveDataIT(&usart2_handle, (uint8_t *) rx_buf, strlen(msg[count])) != USART_READY);
        // Send the message indexed by count in blocking mode
        USART_SendData(&usart2_handle, (uint8_t *) msg[count], strlen(msg[count]));

        // Wait until all the bytes are received from the arduino
        // WHen all the bytes are received rxCmplt will be SET in application callback
        while (rxCmplt != SET);

        rxCmplt = RESET;

        count++;
    }
        return 0;
}

extern "C"{
    void USART2_IRQHandler(){
        USART_IRQHandling(&usart2_handle);
    }
}

void USART_ApplicationEventCallBack(USART_Handle_t *pUSARTHandle, uint8_t app_event){
    if (app_event == USART_EVENT_RX_CMPLT) {
        rxCmplt = SET;
    } else if (app_event == USART_EVENT_TX_CMPLT) {

    }
}