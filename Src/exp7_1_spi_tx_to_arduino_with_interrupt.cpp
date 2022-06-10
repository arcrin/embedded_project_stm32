//
// Created by wbai on 6/8/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"

SPI_Handle_t spi3_handle;

volatile uint8_t send_spi_message = 0;

void SPI3_GPIOInits(){
    GPIO_Handle_t SPIPins;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // NSS: PA4
    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);

    // SCK: PC10
    SPIPins.pGPIOx = GPIOC;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&SPIPins);

    // MISO: PC11
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&SPIPins);

    // MOSI: PC12
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI3_Inits(){
    spi3_handle.pSPIx = SPI3;
    spi3_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi3_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi3_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    spi3_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi3_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi3_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi3_handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
    SPI_Init(&spi3_handle);
}

void GPIO_Button_Init(){
    GPIO_Handle_t b1_gpio_handle;
    b1_gpio_handle.pGPIOx = GPIOA;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_RT_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&b1_gpio_handle);
}

char user_data[] = "Hello World";

int main(){
    GPIO_Button_Init();
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
    GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);

    SPI3_GPIOInits();
    SPI3_Inits();
    SPI_SSOEConfig(SPI3, ENABLE);
    SPI_IRQITConfig(IRQ_NO_SPI3, ENABLE);
    SPI_IRQPriorityConfig(IRQ_NO_SPI3, 15);

    SysTick_Init(16000);

    while(1){
        if(send_spi_message){
            SPI_PeriControl(SPI3, ENABLE);
            uint8_t dataLen = strlen(user_data);
            SPI_Send_Data_IT(&spi3_handle, &dataLen, 1);
            while (SPI_GetFlagStatus(SPI3, SPI_BUSY_FLAG)); // is this necessary
            delay(100);
            SPI_Send_Data_IT(&spi3_handle, (uint8_t *) user_data, dataLen);
            while (SPI_GetFlagStatus(SPI3, SPI_BUSY_FLAG));
            SPI_PeriControl(SPI3, DISABLE);
        }
    }
}

extern "C"{
    void SPI3_IRQHandler(){
        SPI_IRQHandling(&spi3_handle);
        send_spi_message = 0;
    }
}

extern "C" {
    void EXTI0_IRQHandler() {
        GPIO_IRQHandling(0);
        send_spi_message = 1;
    }
}