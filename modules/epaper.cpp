//
// Created by wbai on 1/11/2022.
//

#ifndef MCU1_EXP7_EPAPER_CPP
#define MCU1_EXP7_EPAPER_CPP

#include "epaper.h"
#include "stm32f407xx_spi_driver.h"

const unsigned char WF_PARTIAL[159] ={
        0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x40,0x40,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x0A,0x00,0x00,0x00,0x00,0x00,0x02,0x01,0x00,0x00,
        0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x22,0x22,0x22,0x22,0x22,0x22,
        0x00,0x00,0x00,0x22,0x17,0x41,0xB0,0x32,0x36,
};

/*
 * GPIO pin set up
 */
void EPD_GPIO_Init(){
    GPIO_Handle_t epd_gpio_handle;
    epd_gpio_handle.pGPIOx = GPIOB;
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // RST pin
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&epd_gpio_handle);

    // DC pin
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&epd_gpio_handle);

    // BUSY pin
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&epd_gpio_handle);

    // CS pin
    epd_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&epd_gpio_handle);
}

/*
 * SPI set up
 */
void EPD_SPI2_Init(){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // NSS
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//    GPIO_Init(&SPIPins);

    // SCK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    SPI_Handle_t spi2_handle;
    spi2_handle.pSPIx = SPI2;
    spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    spi2_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
    SPI_Init(&spi2_handle);
    SPI_SSIConfig(SPI2, ENABLE);
//    SPI_SSOEConfig(SPI2, DISABLE);
//    SPI_PeriControl(SPI2, ENABLE);
}

/*
 * EPD functions
 */
static void EPD_2IN66_Reset(){
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(2);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
}


static void EPD_2IN66_SendCommand(uint8_t Reg){
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void EPD_2IN66_SendData(uint8_t Data){
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void EPD_2IN66_ReadBusy(){
    DEV_Delay_ms(100);
    while (DEV_Digital_Read(EPD_BUSY_PIN) == 1) {
        DEV_Delay_ms(100);
    }
    DEV_Delay_ms(100);
}

static void EPD_2IN66_TurnOnDisplay(){
    EPD_2IN66_SendCommand(0x20);
    EPD_2IN66_ReadBusy();
}

static void EPD_2IN66_SetLUA(){
    uint16_t count;
    EPD_2IN66_SendCommand(0x32);
    for (count = 0; count < 153; count++) {
        EPD_2IN66_SendData(WF_PARTIAL[count]);
    }
    EPD_2IN66_ReadBusy();
}

void EPD_2IN66_Init(){
    EPD_2IN66_Reset();
    EPD_2IN66_ReadBusy();
    EPD_2IN66_SendCommand(0x12); // soft reset
    EPD_2IN66_ReadBusy();

    EPD_2IN66_SendCommand(0x11); // data entry mode
    EPD_2IN66_SendData(0x03);
    /* Set RamX-address Start/End position*/
    EPD_2IN66_SendCommand(0x44);
    EPD_2IN66_SendData(0x1);
    EPD_2IN66_SendData((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1));
    /* Set RamY-address Start/End position*/
    EPD_2IN66_SendCommand(0x45);
    EPD_2IN66_SendData(0);
    EPD_2IN66_SendData(0);
    EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0xFF));
    EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0x100) >> 8);

    EPD_2IN66_ReadBusy();
}

void EPD_2IN66_Init_Partial(){
    EPD_2IN66_Reset();
    EPD_2IN66_ReadBusy();
    EPD_2IN66_SendCommand(0x12);
    EPD_2IN66_ReadBusy();

    EPD_2IN66_SetLUA();
    EPD_2IN66_SendCommand(0x37);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x40);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);
    EPD_2IN66_SendData(0x00);

    /* Y increment, X increment */
    EPD_2IN66_SendCommand(0x11);
    EPD_2IN66_SendData(0x03);
    /* Set RamX-address Start/End position */
    EPD_2IN66_SendCommand(0x44);
    EPD_2IN66_SendData(0x01);
    EPD_2IN66_SendData((EPD_2IN66_WIDTH % 8 == 0)? (EPD_2IN66_WIDTH / 8 ): (EPD_2IN66_WIDTH / 8 + 1) );
    /* Set RamY-address Start/End position */
    EPD_2IN66_SendCommand(0x45);
    EPD_2IN66_SendData(0);
    EPD_2IN66_SendData(0);
    EPD_2IN66_SendData((EPD_2IN66_HEIGHT&0xff));
    EPD_2IN66_SendData((EPD_2IN66_HEIGHT&0x100)>>8);

    EPD_2IN66_SendCommand(0x3C);
    EPD_2IN66_SendData(0x80);

    EPD_2IN66_SendCommand(0x22);
    EPD_2IN66_SendData(0xcf);
    EPD_2IN66_SendCommand(0x20);
    EPD_2IN66_ReadBusy();
}

void EPD_2IN66_Clear(void)
{
    uint16_t Width, Height;
    Width = (EPD_2IN66_WIDTH % 8 == 0)? (EPD_2IN66_WIDTH / 8 ): (EPD_2IN66_WIDTH / 8 + 1);
    Height = EPD_2IN66_HEIGHT;
    EPD_2IN66_SendCommand(0x24);
    for (uint16_t j = 0; j <=Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN66_SendData(0xff);
        }
    }
    EPD_2IN66_TurnOnDisplay();
}

void EPD_2IN66_Display(uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD_2IN66_WIDTH % 8 == 0)? (EPD_2IN66_WIDTH / 8 ): (EPD_2IN66_WIDTH / 8 + 1);
    Height = EPD_2IN66_HEIGHT;

    uint32_t Addr = 0;

    // UDOUBLE Offset = ImageName;
    EPD_2IN66_SendCommand(0x24);
    for (uint16_t j = 0; j <Height; j++) {
        for (uint16_t i = 0; i <Width; i++) {
            Addr = i + j * Width;
            EPD_2IN66_SendData(Image[Addr]);
        }
    }
    EPD_2IN66_TurnOnDisplay();
}

void EPD_2IN66_Sleep(){
    EPD_2IN66_SendCommand(0x10);
    EPD_2IN66_SendData(0x01);
}

void DEV_SPI_WriteByte(uint8_t value){
    SPI_PeriControl(SPI2, ENABLE);
    SPI_Send_Data(SPI2, &value, 1);
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
    SPI_PeriControl(SPI2, DISABLE);
}

int DEV_Module_Init(){
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    return 0;
}

void DEV_Module_Exit(){
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    // close 5v?
    DEV_Digital_Write(EPD_RST_PIN, 0);
}

#endif //MCU1_EXP7_EPAPER_CPP
