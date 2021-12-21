//
// Created by wbai on 10/28/2021.
//
#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(pGPIO_RegDef_t pGPIOx,uint8_t EnorDi){
    if (EnorDi == ENABLE) {
        if(pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOB) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOC) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOD) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOE) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOF) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOG) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOH) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOI) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOJ) {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOK) {
            GPIOA_PCLK_EN();
        }
    } else if (EnorDi == DISABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOB) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOC) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOD) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOE) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOF) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOG) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOH) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOI) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOJ) {
            GPIOA_PCLK_DI();
        }
        if (pGPIOx == GPIOK) {
            GPIOA_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */
/********************************************************
 * @fn				- GPIO_Init
 *
 * @brief			-
 *
 * @param[pGPIO_Handle_t]
 *                  -User defined GPIO_Handle_t
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_Init(pGPIO_Handle_t pGPIOHandle){
    //1. configure the mode of gpio pin
    if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE) {
        pGPIOHandle->pGPIOx->MODER &=
                (0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |=
                (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    } else{}

    //2. configure the speed
    pGPIOHandle->pGPIOx->OSPEEDR &=
            ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    pGPIOHandle->pGPIOx->OSPEEDR |=
            (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

    //3. configure the pupd settings
    pGPIOHandle->pGPIOx->PUPDR &=
            ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |=
            (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    //4. configure the optype
    pGPIOHandle->pGPIOx->OTYPER &=
            ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |=
            (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    //5. configure the al functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALTFn_MODE) {
        uint8_t reg_level = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        uint8_t reg_shift = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[reg_level] &=
                ~(0xf << (reg_shift * 4));
        pGPIOHandle->pGPIOx->AFR[reg_level] |=
                (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (reg_shift * 4));
    }
}

void GPIO_DeInit(pGPIO_RegDef_t pGpioRegDef){
    if (pGpioRegDef == GPIOA) {
        GPIOA_REG_RESET();
    }  else if (pGpioRegDef == GPIOB) {
        GPIOB_REG_RESET();
    }  else if (pGpioRegDef == GPIOC) {
        GPIOC_REG_RESET();
    }  else if (pGpioRegDef == GPIOD) {
        GPIOD_REG_RESET();
    }  else if (pGpioRegDef == GPIOE) {
        GPIOE_REG_RESET();
    }  else if (pGpioRegDef == GPIOF) {
        GPIOF_REG_RESET();
    }  else if (pGpioRegDef == GPIOG) {
        GPIOG_REG_RESET();
    }  else if (pGpioRegDef == GPIOH) {
        GPIOH_REG_RESET();
    }  else if (pGpioRegDef == GPIOI) {
        GPIOI_REG_RESET();
    }  else if (pGpioRegDef == GPIOJ) {
        GPIOJ_REG_RESET();
    }  else if (pGpioRegDef == GPIOK) {
        GPIOK_REG_RESET();
    }
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber){
    uint8_t value;
    value = (GPIOx->IDR & (1 << PinNumber)) >> PinNumber;
    return value;
}

uint16_t GPIO_ReadFromInputPortr(pGPIO_RegDef_t GPIOx);
void GPIO_WriteOutputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(pGPIO_RegDef_t GPIOx, uint16_t value);
void GPIO_ToggleOutputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber);

/*
 * Interrupt
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);