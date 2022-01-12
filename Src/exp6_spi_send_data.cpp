//
// Created by wbai on 1/5/2022.
//

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

void SPI2_GPIOInits(void){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFn_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // TODO: why push pull
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // MED or SLOW speed would also work, not sure about VHIGH

    // NSS configure
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

    // SCK configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MISO configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // MOSI configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(){
    SPI_Handle_t spi2_handle;
    spi2_handle.pSPIx = SPI2;
    spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
    spi2_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management
    SPI_Init(&spi2_handle);
}

int main() {
    char user_data[] = "Hello World";
    // enable GPIO pins as SPI alt function
    SPI2_GPIOInits();

    // initialize SPI2, this needs to be done while SPI2 is disabled, CR1->SPE set to 0 (default)
    SPI2_Inits();

    // enable SPI2, CR1->SPE set to 1
    SPI_PeriControl(SPI2, ENABLE);

    // set SSI bit in CR1, without setting this bit, MODF error will occur
    SPI_SSIConfig(SPI2, ENABLE);

    SPI_Send_Data(SPI2, (uint8_t *) user_data, strlen(user_data));

    while(1);

}