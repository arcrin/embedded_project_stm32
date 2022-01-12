//
// Created by wbai on 1/11/2022.
//

#include "EPD_2in66.h"
#include "stm32f407xx_spi_driver.h"

const unsigned char WF_PARTIAL[159] = {
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

void DEV_Delay(uint8_t scale){
    for (int i = 0; i < 500000/scale; i++);
}

void EPD_SPI_GPIOInits(){
    // use GPIOB pin 12, 13, 15 for SPI communication
    GPIO_Handle_t SPI2Pins;
    SPI2Pins.pGPIOx = GPIOB;
    SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFn_MODE;
    SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // NSS configure
    SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPI2Pins);

    // SCK configuration
    SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI2Pins);

    // MOSI configuration
    SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI2Pins);
}

void EPD_ControlPinInit(){
    GPIO_Handle_t control_pins;
    control_pins.pGPIOx = GPIOA;
    control_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    control_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    control_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    control_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // DC pin configure
    control_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Init(&control_pins);

    // RST pin configure
    control_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&control_pins);


    // BUSY pin configure
    control_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    control_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    control_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    control_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&control_pins);
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
    DEV_Digital_Write(EPD_RST_PIN, 0);
}

void DEV_SPI_WriteByte(UBYTE value) {
    SPI_Send_Data(SPI2, &value, 1);
}

/*********************************
 * function: software reset
 * parameter:
 *********************************/
static void EPD_2IN66_Reset(){
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay(10);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay(10);
    DEV_Digital_Write(EPD_RST_PIN, 1);
}

/********************************
 * function: send command
 * parameter:
 *      Reg : Command register
 ********************************/
static void EPD_2IN66_SendCommand(UBYTE Reg){
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/**********************************
 * function : send data
 * parameter:
 *      Data : Write data
 **********************************/
static void EPD_2IN66_SendData(UBYTE Data){
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/*************************************************
 * function : Wait until the busy_pin goes LOW
 * parameter:
 *************************************************/
void EPD_2IN66_ReadBusy(){
    DEV_Delay(20);
    while(DEV_Digital_Read(EPD_BUSY_PIN) == 1) {
        DEV_Delay(20);
    }
    DEV_Delay(20);
}


/*****************************************************
* function : Turn on display
* parameter:
******************************************************/
static void EPD_2IN66_TurnOnDisplay(){
    EPD_2IN66_SendCommand(0x20);
    EPD_2IN66_ReadBusy();
}

/*****************************************************
 * function : send LUT
 * parameter:
 *****************************************************/
 static void EPD_2IN66_SetLUT(){
     UWORD count;
    EPD_2IN66_SendCommand(0x32);
    for(count = 0; count < 153; count++){
        EPD_2IN66_SendData(WF_PARTIAL[count]);
    }
    EPD_2IN66_ReadBusy();
 }

 /********************************************************
  * function : Initialize the e-paper register
  * parameter:
  ********************************************************/
  void EPD_2IN66_Init(){
     EPD_2IN66_Reset();
     EPD_2IN66_ReadBusy();
     EPD_2IN66_SendCommand(0x12);
     EPD_2IN66_ReadBusy();
     // Y increment, X increment
     EPD_2IN66_SendCommand(0x11);
     EPD_2IN66_SendData(0x03);
     // Set RamX-address Start/End position
     EPD_2IN66_SendCommand(0x44);
     EPD_2IN66_SendData(0x01);
     EPD_2IN66_SendData((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1));
     // Set RamY-address Start/End position
     EPD_2IN66_SendCommand(0x45);
     EPD_2IN66_SendData(0);
     EPD_2IN66_SendData(0);
     EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0xff));
     EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0x100) >> 8);
     EPD_2IN66_ReadBusy();
  }

  /***************************************************************
   * function : Initialize the e-paper register (Partial display)
   * parameter:
   **************************************************************/
   void EPD_2IN66_Init_Partial() {
      EPD_2IN66_Reset();
      EPD_2IN66_ReadBusy();
      EPD_2IN66_SendCommand(0x12);
      EPD_2IN66_ReadBusy();

      EPD_2IN66_SetLUT();
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

      // Y increment, X increment
      EPD_2IN66_SendCommand(0x11);
      EPD_2IN66_SendData(0x03);
      // Set RamX-address Start/End position
      EPD_2IN66_SendCommand(0x44);
      EPD_2IN66_SendData(0x01);
      EPD_2IN66_SendData((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1));
      // Set RamY-address Start/End position
      EPD_2IN66_SendCommand(0x45);
      EPD_2IN66_SendData(0);
      EPD_2IN66_SendData(0);
      EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0xff));
      EPD_2IN66_SendData((EPD_2IN66_HEIGHT & 0x100) >> 8);

      EPD_2IN66_SendCommand(0x3C);
      EPD_2IN66_SendData(0x80);

      EPD_2IN66_SendCommand(0x22);
      EPD_2IN66_SendData(0xcf);
      EPD_2IN66_SendCommand(0x20);
      EPD_2IN66_ReadBusy();
   }

   /******************************************
    * function : Clear screen
    * parameter:
    ******************************************/
    void EPD_2IN66_Clear(){
       UWORD Width, Height;
       Width = (EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1);
       Height = EPD_2IN66_HEIGHT;
       EPD_2IN66_SendCommand(0x24);
       for (UWORD j = 0; j <= Height; j++) {
           for (UWORD i = 0; i < Width; i++) {
               EPD_2IN66_SendData(0xff);
           }
       }
       EPD_2IN66_TurnOnDisplay();
    }

/********************************************************************
 * function : Sends the image buffer in RAM to e-paper and displays
 * parameter:
 ********************************************************************/
void EPD_2IN66_Display(UBYTE *Image){
    UWORD Width, Height;
    Width = (EPD_2IN66_WIDTH % 8 == 0)? (EPD_2IN66_WIDTH / 8 ): (EPD_2IN66_WIDTH / 8 + 1);
    Height = EPD_2IN66_HEIGHT;

    UDOUBLE Addr = 0;

    EPD_2IN66_SendCommand(0x24);
    for (UWORD j = 0; j <Height; j++) {
        for (UWORD i = 0; i <Width; i++) {
            Addr = i + j * Width;
            EPD_2IN66_SendData(Image[Addr]);
        }
    }
    EPD_2IN66_TurnOnDisplay();
}

/*********************************************************************
 * function : Enter sleep mode
 * parameter:
 *********************************************************************/
void EPD_2IN66_Sleep() {
    EPD_2IN66_SendCommand(0x10);
    EPD_2IN66_SendData(0x01);
}