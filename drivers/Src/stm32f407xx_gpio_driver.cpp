//
// Created by wbai on 12/29/2021.
//

/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct. 4, 2021
 *      Author: Wen
 */
#include <cstdint>
#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral Clock setup
 */

/********************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[pGPIO_Handle_t]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
    }
}

/*
 * Init and De-Init
 */
/********************************************************
 * @fn				- GPIO_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;
    // 1. configure the mode of gpio pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE)
    {
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
                << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    }else
    {
        // Interrupt
    }
    temp = 0;
    // 2. configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;
    // 3. configure the pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;
    // 4. configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;
    // 5. configure the alt functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALTFN_MODE)
    {
        // configure the alt function registers
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

/********************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    if(pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    if(pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    if(pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    if(pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    if(pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    if(pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    if(pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    if(pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*
 * Data read and write
 */

/********************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- None
 ********************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

/********************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- None
 ********************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/********************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- None
 ********************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        // write 1 to the output data register at the bit field corresponding pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }else
    {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/********************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- None
 ********************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/********************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- None
 ********************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber); // XOR operation
}