//
// Created by wbai on 6/7/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"


SPI_Handle_t SPI2Handle;

#define MAX_LEN     500

char RcvBuffer[MAX_LEN];

uint8_t ReadByte;

volatile uint8_t rcvStop = 0;

/* This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void){
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void){
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){
    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA - SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI2Handle);
}

/* This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(){
    GPIO_Handle_t spiIntPin;
    memset(&spiIntPin, 0, sizeof(spiIntPin));

    // this is led gpio configuration
    spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_FT_MODE;
    spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&spiIntPin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(){
    uint8_t dummy = 0xff;
    Slave_GPIO_InterruptPinInit();

    // this function is used to initialize the GPIO pins behave as SPI2 pins
    SPI2_GPIOInits();

    // this function is used to initialize the SPI2 peripheral parameters
    SPI2_Inits();

    /*
     * making SSOE 1 does NSS output enable.
     * The NSS pin is automatically managed by the hardware.
     * i.e. when SPE=1, NSS will be pulled to low
     * and NSS pin will be high when SPE=0
     */
    SPI_SSOEConfig(SPI2, ENABLE);
    SPI_IRQITConfig(IRQ_NO_SPI2, ENABLE);

    while(1){
        rcvStop = 0;

        while(!dataAvailable); // wait till data available interrupt from transmitter device(slave)

        GPIO_IRQITConfig(IRQ_NO_EXTI9_5, DISABLE);

        // enable the SPI2 peripheral
        SPI_PeriControl(SPI2, ENABLE);

        while (!rcvStop) {
            /*
             * fetch the data from the SPI peripheral byte by byte in interrupt mode
             */
            while(SPI_Send_Data_IT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
            while(SPI_Receive_Data_IT(&SPI2Handle, &ReadByte, 1) == SPI_BUSY_IN_RX);
        }

        // confirm SPI is not busy
        while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
        // Disable the SPI2 peripheral
        SPI_PeriControl(SPI2, DISABLE);

        // printf("Rcv data = %s\n, RcvBuff);

        dataAvailable = 0;

        GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
    }
    return 0;
}

void SPI2_IRQHandler(void){
    SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

}