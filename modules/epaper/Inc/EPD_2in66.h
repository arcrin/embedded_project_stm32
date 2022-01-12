//
// Created by wbai on 1/11/2022.
//

#ifndef MCU1_EPD_2IN66_H
#define MCU1_EPD_2IN66_H

#include <cstdint>
#include "stm32f407xx_gpio_driver.h"


/*
 * data
 */
#define UBYTE       uint8_t
#define UWORD       uint16_t
#define UDOUBLE    uint32_t


/*
 * e-paper
 */
#define EPD_RST_PIN       GPIOA, GPIO_PIN_NO_0
#define EPD_DC_PIN        GPIOA, GPIO_PIN_NO_1
#define EPD_CS_PIN        GPIOA, GPIO_PIN_NO_2
#define EPD_BUSY_PIN      GPIOA, GPIO_PIN_NO_3

/*
 * GPIO read and Write
 */
#define DEV_Digital_Write(_pin, _value) GPIO_WriteOutputPin(_pin, _value==0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define DEV_Digital_Read(_pin)          GPIO_ReadFromInputPin(_pin)

/*
 * Display resolution
 */
#define EPD_2IN66_WIDTH     152
#define EPD_2IN66_HEIGHT    296

void DEV_Delay(uint8_t scale);

void DEV_SPI_WriteByte(UBYTE value);

int DEV_Module_Init();

void DEV_Module_Exit();

void EPD_2IN66_Init();

void EPD_2IN66_Init_Partial();

void EPD_2IN66_Clear();

void EPD_2IN66_Dispalay(UBYTE *Image);

void EPD_2IN66_Sleep();

#endif //MCU1_EPD_2IN66_H
