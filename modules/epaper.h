//
// Created by wbai on 5/10/2022.
//

#ifndef MCU1_EPAPER_H
#define MCU1_EPAPER_H
#include "stm32f407xx.h"

#define RST_Pin             GPIO_PIN_NO_8
#define RST_GPIO_Port       GPIOB
#define DC_Pin              GPIO_PIN_NO_9
#define DC_GPIO_Port        GPIOB
#define BUSY_Pin            GPIO_PIN_NO_10
#define BUSY_GPIO_Port      GPIOB
#define SPI_CS_Pin          GPIO_PIN_NO_12
#define SPI_CS_GPIO_Port    GPIOB

/*
 * e-Paper GPIO
 */
#define EPD_RST_PIN     RST_GPIO_Port, RST_Pin
#define EPD_DC_PIN      DC_GPIO_Port, DC_Pin
#define EPD_CS_PIN      SPI_CS_GPIO_Port, SPI_CS_Pin
#define EPD_BUSY_PIN    BUSY_GPIO_Port, BUSY_Pin

/*
 * GPIO read and write
 */
#define DEV_Digital_Write(_pin, _value) GPIO_WriteOutputPin(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define DEV_Digital_Read(_pin) GPIO_ReadFromInputPin(_pin)

/*
 * delay
 */
#define DEV_Delay_ms(__xms)     delay(__xms)

void DEV_SPI_WriteByte(uint8_t value);

int DEV_Module_Init();

void DEV_Module_Exit();


/*
 * 2.66 inch epaper display specific
 */
#define EPD_2IN66_WIDTH        152
#define EPD_2IN66_HEIGHT       296

void EPD_2IN66_Init();

void EPD_2IN66_Init_Partial();

void EPD_2IN66_Clear();

void EPD_2IN66_Display(uint8_t *Image);

void EPD_2IN66_Sleep();

void EPD_GPIO_Init();

void EPD_SPI2_Init();



#endif //MCU1_EPAPER_H
