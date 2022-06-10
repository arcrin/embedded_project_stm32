//
// Created by wbai on 6/7/2022.
//
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "stm32f407xx.h"


SPI_Handle_t SPI2Handle;

SPI_Handle_t spi3_handle;

#define MAX_LEN     500

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;

/* This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

//void delay(void){
//    for (uint32_t i = 0; i < 500000 / 2; i++);
//}



//void SPI2_EPAPER_GPIOInits(void){
//    /*
//     * epaper
//     * PB14 --> SPI2_MISO
//     * PB15 --> SPI2_MOSI
//     * PB13 -> SPI2_SCLK
//     * PB12 --> SPI2_NSS
//     * ALT function mode : 5
//     */
//
//    GPIO_Handle_t SPIPins;
//
//    SPIPins.pGPIOx = GPIOB;
//    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
//    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
//    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//
//    // SCLK
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//    GPIO_Init(&SPIPins);
//
//    // MOSI
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
//    GPIO_Init(&SPIPins);
//
//    // MISO
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_Init(&SPIPins);
//
//    // NSS
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//    GPIO_Init(&SPIPins);
//}
//
//void SPI2_EPAPER_Inits(void){
//    SPI2Handle.pSPIx = SPI2;
//    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
//    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
//    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
//    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
//    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
//    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
//
//    SPI_Init(&SPI2Handle);
//}

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

/* This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(){
    GPIO_Handle_t spi_com_init_pin;
    memset(&spi_com_init_pin, 0, sizeof(spi_com_init_pin));

    // this is led gpio configuration
    spi_com_init_pin.pGPIOx = GPIOD;
    spi_com_init_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    spi_com_init_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_FT_MODE;
    spi_com_init_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spi_com_init_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&spi_com_init_pin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(){
    uint8_t dummy = 0xff;
    Slave_GPIO_InterruptPinInit();

    EPD_GPIO_Init();
    EPD_SPI2_Init();
    SPI_SSOEConfig(SPI2, ENABLE);

    SPI3_GPIOInits();
    SPI3_Inits();
    SPI_SSOEConfig(SPI3, ENABLE);
    SPI_IRQITConfig(IRQ_NO_SPI3, ENABLE);
//    SPI_IRQPriorityConfig(IRQ_NO_SPI3, 15);

    SysTick_Init(16000);

    while(1){
        rcvStop = 0;
        while(!dataAvailable); // wait till data available interrupt from transmitter device(slave)

        GPIO_IRQITConfig(IRQ_NO_EXTI9_5, DISABLE);

        // enable the SPI2 peripheral
        SPI_PeriControl(SPI3, ENABLE);

        while (!rcvStop) {
            /*
             * fetch the data from the SPI peripheral byte by byte in interrupt mode
             * for SPI, master has to send one byte of data to receive one byte of data
             */
            while(SPI_Send_Data_IT(&spi3_handle, &dummy, 1) == SPI_BUSY_IN_TX);
            delay(100);
            while(SPI_Receive_Data_IT(&spi3_handle, (uint8_t *)&ReadByte, 1) == SPI_BUSY_IN_RX);
        }

        // confirm SPI is not busy
        while(SPI_GetFlagStatus(SPI3, SPI_BUSY_FLAG));
        // Disable the SPI2 peripheral
        SPI_PeriControl(SPI3, DISABLE);

        // printf("Rcv data = %s\n, RcvBuff);
        dataAvailable = 0;
        GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
        char *str_to_print;
        for(uint16_t i = 0; i < 500; i++){
            if(RcvBuff[i] != 0xff){
                str_to_print = &RcvBuff[i];
                break;
            }
        }
        delay(100);

        if (DEV_Module_Init() != 0) {
            continue;
        }
        EPD_2IN66_Init();
        EPD_2IN66_Clear();
        delay(5000);

        uint8_t *BlankImage;
        uint16_t ImageSize =
                ((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1)) * EPD_2IN66_HEIGHT;
        if ((BlankImage = (uint8_t *) malloc(ImageSize)) == NULL) {
            return -1;
        }
        Paint_NewImage(BlankImage, EPD_2IN66_WIDTH, EPD_2IN66_HEIGHT, 270, WHITE);
        Paint_SelectImage(BlankImage);
        Paint_Clear(WHITE);
        Paint_DrawString_EN(10, 20, str_to_print, &Font24, BLACK, WHITE);
        delay(200);
        EPD_2IN66_Display(BlankImage);
        delay(5000);
        EPD_2IN66_Sleep();
        free(BlankImage);
        BlankImage = NULL;
        DEV_Module_Exit();
        std::fill_n(RcvBuff, MAX_LEN, 0);
    }
    return 0;
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
    static uint32_t i = 0;
    /*
     * In the RX complete event, copy data in to rcv buffer. '\0' indicates end of message(rcvStop = 1)
     */
    if (AppEv == SPI_EVENT_RX_CMPLT) {
        RcvBuff[i++] = ReadByte;
        if (ReadByte == '\0' || (i == MAX_LEN)) {
            rcvStop = 1;
            RcvBuff[i - 1] = '\0';
            i = 0;
        }
    }
}

/*
 * Slave data available interrupt handler
 */
extern "C"{
    void EXTI9_5_IRQHandler(void){
        GPIO_IRQHandling(GPIO_PIN_NO_6);
        dataAvailable = 1;
    }
}

extern "C"{
    void SPI3_IRQHandler(void){
        SPI_IRQHandling(&spi3_handle);
    }
}