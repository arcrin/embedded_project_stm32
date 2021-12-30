//
// Created by wbai on 12/29/2021.
//

#ifndef MCU1_STM32F407XX_GPIO_DRIVER_H
#define MCU1_STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

/*
 * Handle structure for GPIO pin
 */

typedef struct{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;	/*!<possible values from @GPIO_PIN_MODES>*/
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
    //pointer to hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx;              // holds the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig;     // holds GPIO pin configuration settings
}GPIO_Handle_t, *pGPIO_Handle_t;


/*
 * GPIO PIN definition
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_IN_MODE        0
#define GPIO_OUT_MODE       1
#define GPIO_ALTFN_MODE     2
#define GPIO_ANALOG_MODE    3
#define GPIO_IT_FT_MODE     4   // Interrupt mode falling edge trigger
#define GPIO_IT_RT_MODE     5   // Interrupt mode rising edge trigger
#define GPIO_IT_RFT_MODE    6   // Interrupt mode rising/falling edge trigger

/*
 * GPIO output type @GPIO_PIN_OTYPE
 */
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/*
 * GPIO speed mode @GPIO_PIN_SPEED
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MED      1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_VHIGH    3

/*
 * GPIO PU/PD @GPIO_PIN_PUPDTYPE
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIO, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIODeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

#endif //MCU1_STM32F407XX_GPIO_DRIVER_H
