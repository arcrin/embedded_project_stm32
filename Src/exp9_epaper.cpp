//
// Created by wbai on 5/10/2022.
//

#include "epaper.h"


void GPIO_Button_Init(){
    GPIO_Handle_t b1_gpio_handle;
    b1_gpio_handle.pGPIOx = GPIOA;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&b1_gpio_handle);
}

void SPI2_GPIOInits(void){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFn_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // TODO: why push pull
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // MED or SLOW speed would also work, not sure about VHIGH

//     NSS configure
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

    // SCK configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MISO configuration
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_Init(&SPIPins);

    // MOSI configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(){
    SPI_Handle_t spi2_handle;
    spi2_handle.pSPIx = SPI2;
    spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    spi2_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
    spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin
    SPI_Init(&spi2_handle);
}

int main(){
    SysTick_Init(16000);
    SPI2_GPIOInits();
    SPI2_Inits();
    GPIO_Button_Init();
    SPI_SSOEConfig(SPI2, ENABLE);

    while(1) {
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(100);
//        EPD_2IN66_Clear();
        SPI_PeriControl(SPI2, ENABLE);
        delay(500);
        SPI_PeriControl(SPI2, DISABLE);
    }
}
