//
// Created by andy- on 2021-10-26.
//



#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H

#include <cstdint>

#define _vo     volatile
/*
 * Base memory address
 */
#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM                SRAM1_BASEADDR
#define SRAM2_BASEADDR      (SRAM1_BASEADDR + 0x0001c000)
#define ROM_BASEADDR        0x1FFF0000U  // System memory

/*
 * Peripheral bus base addresses
 */
#define PERIPH_BASE         0x40000000
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     0x40010000
#define AHB1PERIPH_BASE     0x40020000
#define AHB2PERIPH_BASE     0x50000000


/*
 * GPIO ports base addresses
 */
#define GPIOA_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR      (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR      (AHB1PERIPH_BASE + 0x2800)

/*
 * RCC base address
 */
#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x3800)

/*
 * GPIO register structure
 */
typedef struct{
    _vo int32_t MODER;
    _vo int32_t OTYPER;
    _vo int32_t OSPEEDR;
    _vo int32_t PUPDR;
    _vo int32_t IDR;
    _vo int32_t ODR;
    _vo int32_t BSRR;
    _vo int32_t LCKR;
    _vo int32_t AFR[2];
} GPIO_RegDef_t, *pGPIO_RegDef_t;

/*
 * RCC register structure
 */
typedef struct {
    _vo int32_t CR;
    _vo int32_t PLLCFGR;
    _vo int32_t CFGR;
    _vo int32_t CIR;
    _vo int32_t AHB1RSTR;
    _vo int32_t AHB2RSTR;
    _vo int32_t AHB3RSTR;
    int32_t Reserved1;
    _vo int32_t APB1RSTR;
    _vo int32_t APB2RSTR;
    int32_t Reserved2[2];
    _vo int32_t AHB1ENR;
    _vo int32_t AHB2ENR;
    _vo int32_t AHB3ENR;
    int32_t Reserved3;
    _vo int32_t APB1ENR;
    _vo int32_t APB2ENR;
    int32_t Reserved4[2];
    _vo int32_t AHB1LPENR;
    _vo int32_t AHB2LPENR;
    _vo int32_t AHB3LPENR;
    int32_t Reserved5;
    _vo int32_t APB1LPENR;
    _vo int32_t APB2LPENR;
    int32_t Reserved6[2];
    _vo int32_t BDCR;
    _vo int32_t CSR;
    int32_t Reserved7[2];
    _vo int32_t SSCGR;
    _vo int32_t PLLI2SCFGR;
    _vo int32_t PLLSAICFGR;
    _vo int32_t DCKCFGR;
}RCC_RegDef_t, *pRCC_RegDef_t;

/*
 * GPIO definitions
 */
#define GPIOA   ((pGPIO_RegDef_t) GPIOA_BASEADDR)
#define GPIOB   ((pGPIO_RegDef_t) GPIOB_BASEADDR)
#define GPIOC   ((pGPIO_RegDef_t) GPIOC_BASEADDR)
#define GPIOD   ((pGPIO_RegDef_t) GPIOD_BASEADDR)
#define GPIOE   ((pGPIO_RegDef_t) GPIOE_BASEADDR)
#define GPIOF   ((pGPIO_RegDef_t) GPIOF_BASEADDR)
#define GPIOG   ((pGPIO_RegDef_t) GPIOG_BASEADDR)
#define GPIOH   ((pGPIO_RegDef_t) GPIOH_BASEADDR)
#define GPIOI   ((pGPIO_RegDef_t) GPIOI_BASEADDR)
#define GPIOJ   ((pGPIO_RegDef_t) GPIOJ_BASEADDR)
#define GPIOK   ((pGPIO_RegDef_t) GPIOK_BASEADDR)

/*
 * RCC definition
 */
#define RCC     ((pRCC_RegDef_t) RCC_BASEADDR)


/*
 * Clock macros
 */
#define GPIOA_PCLK_EN()     RCC->AHB1ENR |= 1 << 0
#define GPIOB_PCLK_EN()     RCC->AHB1ENR |= 1 << 1
#define GPIOC_PCLK_EN()     RCC->AHB1ENR |= 1 << 2
#define GPIOD_PCLK_EN()     RCC->AHB1ENR |= 1 << 3
#define GPIOE_PCLK_EN()     RCC->AHB1ENR |= 1 << 4
#define GPIOF_PCLK_EN()     RCC->AHB1ENR |= 1 << 5
#define GPIOG_PCLK_EN()     RCC->AHB1ENR |= 1 << 6
#define GPIOH_PCLK_EN()     RCC->AHB1ENR |= 1 << 7
#define GPIOI_PCLK_EN()     RCC->AHB1ENR |= 1 << 8
#define GPIOJ_PCLK_EN()     RCC->AHB1ENR |= 1 << 9
#define GPIOK_PCLK_EN()     RCC->AHB1ENR |= 1 << 10

#define GPIOA_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 8)
#define GPIOJ_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 9)
#define GPIOK_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 10)


/*
 * GPIO reset macro
 */

#define GPIOA_REG_RESET()        do{RCC->AHB1RSTR |= (1<<0); RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOB_REG_RESET()        do{RCC->AHB1RSTR |= (1<<1); RCC->AHB1RSTR &= ~(1<<1);} while(0)
#define GPIOC_REG_RESET()        do{RCC->AHB1RSTR |= (1<<2); RCC->AHB1RSTR &= ~(1<<2);} while(0)
#define GPIOD_REG_RESET()        do{RCC->AHB1RSTR |= (1<<3); RCC->AHB1RSTR &= ~(1<<3);} while(0)
#define GPIOE_REG_RESET()        do{RCC->AHB1RSTR |= (1<<4); RCC->AHB1RSTR &= ~(1<<4);} while(0)
#define GPIOF_REG_RESET()        do{RCC->AHB1RSTR |= (1<<5); RCC->AHB1RSTR &= ~(1<<5);} while(0)
#define GPIOG_REG_RESET()        do{RCC->AHB1RSTR |= (1<<6); RCC->AHB1RSTR &= ~(1<<6);} while(0)
#define GPIOH_REG_RESET()        do{RCC->AHB1RSTR |= (1<<7); RCC->AHB1RSTR &= ~(1<<7);} while(0)
#define GPIOI_REG_RESET()        do{RCC->AHB1RSTR |= (1<<8); RCC->AHB1RSTR &= ~(1<<8);} while(0)
#define GPIOJ_REG_RESET()        do{RCC->AHB1RSTR |= (1<<9); RCC->AHB1RSTR &= ~(1<<9);} while(0)
#define GPIOK_REG_RESET()        do{RCC->AHB1RSTR |= (1<<10); RCC->AHB1RSTR &= ~(1<<10);} while(0)


/*
 * Generic macros
 */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET


#endif //MCU1_STM32F407XX_H
