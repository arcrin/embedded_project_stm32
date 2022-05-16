//
// Created by wbai on 10/28/2021.
//
#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */
/********************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- Initialize or Deinitialize peripheral clock for given GPIO port
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer that contains register definitions for GPIO peripheral
 * @param[uint8_t]  - EN/DI value that we want to assign to the GPIO port
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_PeriClockControl(pGPIO_RegDef_t pGPIOx,uint8_t ENorDI){
    if (ENorDI == ENABLE) {
        if(pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        }
        else if(pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
        else if(pGPIOx == GPIOJ) {
            GPIOJ_PCLK_EN();
        }
        else if(pGPIOx == GPIOK) {
            GPIOK_PCLK_EN();
        }
    } else if (ENorDI == DISABLE) {
        if(pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        }
        else if(pGPIOx == GPIOF) {
            GPIOF_PCLK_DI();
        }
        else if(pGPIOx == GPIOG) {
            GPIOG_PCLK_DI();
        }
        else if(pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        }
        else if(pGPIOx == GPIOI) {
            GPIOI_PCLK_DI();
        }
        else if(pGPIOx == GPIOJ) {
            GPIOJ_PCLK_DI();
        }
        else if(pGPIOx == GPIOK) {
            GPIOK_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */
/********************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize registers that contains a pGPIO_RegDef_t and a GPIO_PinConfig_t
 *
 * @param[pGPIO_Handle_t]
 *                  - GPIO_Handle_t pointer
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void GPIO_Init(pGPIO_Handle_t pGPIOHandle){
    // enable clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    //1. configure the mode of gpio pin
    if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE) {
        pGPIOHandle->pGPIOx->MODER &=
                ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |=
                (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    } else{
        // All these codes are to enable the interrupt on peripheral side
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RT_MODE) {
            // 1. Enable rising edge trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // 2. Disable falling edge trigger
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FT_MODE) {
            // 1. Enable falling edge trigger
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // 2. Disable rising edge trigger
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RFT_MODE) {
            // 1. Enable both rising and falling edge trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        uint8_t exticr_level = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
        uint8_t exticr_bit_position = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;
        SYSCFG_PCLK_EN();
        uint32_t port_code = GET_GPIO_PORT_CODE(pGPIOHandle->pGPIOx);
        SYSCFG->EXTICR[exticr_level] |= (port_code << (exticr_bit_position * 4)); // shift 4 times
        // enable EXTI interrupt delivery
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
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

/********************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- Reset given GPIO port
 *
 * @param[pGPIO_Handle_t]
 *                  - GPIO_RegDef_t pointer
 *
 * @return			- None
 *
 * @Note			- take use of peripheral bus reset registers
 ********************************************************/
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
/********************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Read the value on a given GPIO pin
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer
 * @param[uint8_t]  - pin of interest
 *
 * @return[uint8_t] - value on pin of interest
 *
 * @Note			- reading from IDR
 ********************************************************/
uint8_t GPIO_ReadFromInputPin(pGPIO_RegDef_t GPIOx, uint8_t PinNumber){
    uint8_t value;
    value = (GPIOx->IDR & (1 << PinNumber)) >> PinNumber;
    return value;
}
/********************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Read the value on a given GPIO port
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer
 * @param[uint8_t]  - pin of interest
 *
 * @return[uint16_t]
 *                  - value on port of interest
 *
 * @Note			- reading the entire IDR
 ********************************************************/
uint16_t GPIO_ReadFromInputPort(pGPIO_RegDef_t GPIOx) {
    uint16_t value;
    value = (uint16_t) GPIOx->IDR;
    return value;
}
/********************************************************
 * @fn				- GPIO_WriteOutputPin
 *
 * @brief			- Write a given value to pin of interest
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer
 * @param[uint8_t]  - PinNumber
 * @param[uint8_t]  - value to write, can only be either 1 (GPIO_PIN_SET) or 0 (GPIO_PIN_RESET)
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_WriteOutputPin(pGPIO_RegDef_t pGPIOx, uint8_t PinNumber, uint8_t value){
    if(value == GPIO_PIN_SET){
        pGPIOx->ODR |= (0x1 << PinNumber);
    }
    else if(value == GPIO_PIN_RESET){
        pGPIOx->ODR &= ~(0x1 << PinNumber);
    }
}
/********************************************************
 * @fn				- GPIO_WriteOutputPort
 *
 * @brief			- Write a given value to port of interest
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer
 * @param[uint16_t]  - value to write
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_WriteOutputPort(pGPIO_RegDef_t pGPIOx, uint16_t value){
    pGPIOx->ODR = value;
}
/********************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggle a GPIO pin of interest
 *
 * @param[pGPIO_RegDef_t]
 *                  - GPIO_RegDef_t pointer
 * @param[uint8_t]  - pin number
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_ToggleOutputPin(pGPIO_RegDef_t pGPIOx, uint8_t PinNumber){
    pGPIOx->ODR ^= (0x1 << PinNumber);
}

/*
 * Interrupt
 */
/********************************************************
 * @fn				- GPIO_IRQITConfig
 *
 * @brief			- Enable corresponding interrupt based on given IRQ number, this is considered
 *                    enabling interrupt on processor side
 *
 * @param[uint8_t]  - IRQ number (position)
 * @param[uint8_t]  - ENABLE or DISABLE
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t ENorDI) {
    if (ENorDI == ENABLE) {
        if (IRQNumber <= 31) {
            // Program ISER0 register
            *NVIC_ISER0 |= (0x1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber <= 63) {
            // Program ISER1 register
            *NVIC_ISER1 |= (0x1 << (IRQNumber % 32));
        } else if (IRQNumber > 63 && IRQNumber <= 96) {
            // Program ISER2 register
            *NVIC_ISER2 |= (0x1 << (IRQNumber % 64));
        }
    } else {
        if (IRQNumber <= 31) {
            // Program ISER0 register
            *NVIC_ICER0 |= (0x1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber <= 63) {
            // Program ISER1 register
            *NVIC_ICER1 |= (0x1 << (IRQNumber % 32));
        } else if (IRQNumber > 63 && IRQNumber <= 96) {
            // Program ISER2 register
            *NVIC_ICER2 |= (0x1 << (IRQNumber % 64));
        }
    }
}
/********************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- Set priority for given IRQ number
 *
 * @param[uint8_t]  - IRQ number (position)
 * @param[uint8_t]  - ENABLE or DISABLE
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
    uint8_t ipr_register_offset = IRQNumber / 4;
    uint8_t ipr_byte_offset = IRQNumber % 4;
    *(NVIC_IPR_BASEADDR + (0x4 * ipr_register_offset)) |= ((IRQPriority << NVIC_PRIORITY_BITS) << (ipr_byte_offset * 8));
}
/********************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- Set priority for given IRQ number
 *
 * @param[uint8_t]  - GPIO pin number
 *
 * @return          - None
 *
 * @Note			-
 ********************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){
    // clear the exti PR (pending register) register
    if (EXTI->PR & (1 << PinNumber)) {
        EXTI->PR |= (1 << PinNumber);
    }
}