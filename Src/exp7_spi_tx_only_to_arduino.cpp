//
// Created by wbai on 1/5/2022.
//
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <cstring>

/*
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * Alternate function: AF5
 */

//void delay(void)
//{
//    for(uint32_t i = 0 ; i < 500000 ; i ++);
//}

void SPI2_GPIOInits(void){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
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
    spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin
    SPI_Init(&spi2_handle);
}

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

int main() {
    char user_data[] = "Hello World";
    // enable GPIO pins as SPI alt function
    SPI2_GPIOInits();

    // initialize SPI2, this needs to be done while SPI2 is disabled, CR1->SPE set to 0 (default)
    SPI2_Inits();

    GPIO_Button_Init();

    SPI_SSOEConfig(SPI2, ENABLE);

//    SPI_PeriControl(SPI2, ENABLE);

    SysTick_Init(16000);

    while (1) {
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay(100);

        SPI_PeriControl(SPI2, ENABLE);

        uint8_t dataLen = strlen(user_data);
        SPI_Send_Data(SPI2, &dataLen, 1);

        // send data
        SPI_Send_Data(SPI2, (uint8_t *) user_data, strlen(user_data));

        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        // disable SPI2
        SPI_PeriControl(SPI2, DISABLE);
    }

}