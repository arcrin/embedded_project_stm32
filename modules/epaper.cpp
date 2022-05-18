//
// Created by wbai on 1/11/2022.
//

#ifndef MCU1_EXP7_EPAPER_CPP
#define MCU1_EXP7_EPAPER_CPP

#include "epaper.h"


static void EPD_2IN66B_Reset(){
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(2);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
}


static void EPD_2IN66B_SendCommand(uint8_t Reg){
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void EPD_2IN66B_SendData(uint8_t Data){
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void EPD_2IN66B_ReadyBusy(){
    DEV_Delay_ms(50);
    while (DEV_Digital_Read(EPD_BUSY_PIN) == 1) {
        DEV_Delay_ms(10);
    }
    DEV_Delay_ms(50);
}

static void EPD_2IN66B_TurnOnDisplay(){
    EPD_2IN66B_SendCommand(0x20);
    EPD_2IN66B_ReadyBusy();
}


static void EPD_2In66B_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend){
    EPD_2IN66B_SendCommand(0x44); //SET_RAM_X_ADDRESS_START_END_POSITION
    EPD_2IN66B_SendData((Xstart >> 3) & 0x1F);
    EPD_2IN66B_SendData((Xend >> 3) & 0x1F);

    EPD_2IN66B_SendCommand(0x45); // SET_RAM_Y_ADDRESS_START_END_POSITION
    EPD_2IN66B_SendData(Ystart & 0xFF);
    EPD_2IN66B_SendData((Ystart >> 8) & 0xFF);
    EPD_2IN66B_SendData(Yend & 0xFF);
    EPD_2IN66B_SendData((Yend >> 8) & 0x01);
}

static void EPD_2IN66B_SetCursor(uint16_t Xstart, uint16_t Ystart){
    EPD_2IN66B_SendCommand(0x4E); // SET_RAN_X_ADDRESS_COUNTER
    EPD_2IN66B_SendData(Xstart & 0x1F);

    EPD_2IN66B_SendCommand(0x4F);
    EPD_2IN66B_SendData(Ystart & 0xFF);
    EPD_2IN66B_SendData((Ystart >> 8) & 0x01);
}

void EPD_2IN66B_Init(){
    EPD_2IN66B_Reset();
    EPD_2IN66B_ReadyBusy();
    EPD_2IN66B_SendCommand(0x12); // soft reset
    EPD_2IN66B_ReadyBusy();

    EPD_2IN66B_SendCommand(0x11); // data entry mode
    EPD_2IN66B_SendData(0x03);

    EPD_2IN66B_SendCommand(0x21);
    EPD_2IN66B_SendData(0x00);
    EPD_2IN66B_SendData(0x80);

    EPD_2IN66B_SetCursor(0, 0);
    EPD_2IN66B_ReadyBusy();
}

void EPD_2IN66B_Display(uint8_t *ImageBlack, uint8_t *ImageRed){
    uint16_t Width, Height;
    Width = (EPD_2IN66B_WIDTH % 8 == 0) ? (EPD_2IN66B_WIDTH / 8) : EPD_2IN66B_WIDTH / 8 + 1;
    Height = EPD_2IN66B_HEIGHT;

    EPD_2IN66B_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN66B_SendData(ImageBlack[i + j * Width]);
        }
    }

    EPD_2IN66B_SendCommand(0x26);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN66B_SendData(~ImageRed[i + j * Width]);
        }
    }
    EPD_2IN66B_TurnOnDisplay();
}

void EPD_2IN66B_Clear(){
    uint16_t Width, Height;
    Width = (EPD_2IN66B_WIDTH % 8 == 0) ? (EPD_2IN66B_WIDTH / 8) : EPD_2IN66B_WIDTH / 8 + 1;
    Height = EPD_2IN66B_HEIGHT;

    EPD_2IN66B_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN66B_SendData(0xFF);
        }
    }
    EPD_2IN66B_SendCommand(0x26);
    for (uint8_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN66B_SendData(0x00);
        }
    }
    EPD_2IN66B_TurnOnDisplay();
}

void EPD_2IN66B_Sleep(){
    EPD_2IN66B_SendCommand(0x10);
    EPD_2IN66B_SendData(0x01);
}

#endif //MCU1_EXP7_EPAPER_CPP
