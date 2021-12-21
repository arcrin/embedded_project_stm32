//
// Created by andy- on 2021-12-05.
//

/*
 * this file contains driver specific data, such as:
 * GPIO handle and configuration structures that can be used by user application
 */

#include "stm32f407xx.h"
typedef struct { // haven't seen this being used in pointer form, my guess is that there is no need to alter the value of this struct
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;       // Possible modes @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
    //pointer to hold the base address of the GPIO peripheral
    pGPIO_RegDef_t pGPIOx;              // holds the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig;     // holds GPIO pin configuration settings
}GPIO_Handle_t, *pGPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_IN_MODE        0
#define GPIO_OUT_MODE       1
#define GPIO_ALTFn_MODE     2
#define GPIO_ANALOG_MODE    3
#define GPIO_IT_FT_MODE     4   // Interrupt mode falling edge trigger
#define GPIO_IT_RT_MODE     5   // Interrupt mode rising edge trigger
#define GPIO_IT_RFT_MODE    6   // Interrupt mode rising/falling edge trigger

/*
 * GPIO output type
 */
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/*
 * GPIO speed mode
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MED      1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_VHIGH    3

/*
 * GPIO PU/PD
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2

/*******************************************************************************
 * APIs supported by this driver
 *******************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(pGPIO_RegDef_t pGPIOx,uint8_t ENorDi);

/*
 * Init and De-init
 */
void GPIO_Init(pGPIO_Handle_t pGPIOHandle);
void GPIO_DeInit(pGPIO_RegDef_t pGpioRegDef);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPortr(pGPIO_RegDef_t GPIOx);
void GPIO_WriteOutputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(pGPIO_RegDef_t GPIOx, uint16_t value);
void GPIO_ToggleOutputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber);

/*
 * Interrupt
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);




