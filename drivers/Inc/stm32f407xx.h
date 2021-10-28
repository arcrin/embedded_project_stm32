//
// Created by andy- on 2021-10-26.
//



#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H

#include <stdint.h>

#define __vo    volatile

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM1_SIZE          112 * 1024
#define SRAM2               SRAM1_BASEADDR + SRAM1_SIZE // 0X2001C000
#define ROM                 0x1FFF0000U     // system memory
#define SRAM                SRAM1_BASEADDR

/*
 *  AHBx amd APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASE
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U

/*
 * Peripheral base addresses on AHB1 bus
 *
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

#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x3800)


/*
 * register structure definition for GPIO
 */
typedef struct
{
    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct {
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    __vo uint32_t RESERVE1;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    __vo uint32_t RESERVE2[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    __vo uint32_t RESERVE3;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    __vo uint32_t RESERVE4[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    __vo uint32_t RESERVE5;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    __vo uint32_t RESERVE6[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    __vo uint32_t RESERVE7[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * Peripheral definitions
 */
#define GPIOA       ((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_Regdef_t*)GPIOH_BASEADDR)
#define GPIOI       ((GPIO_Regdef_t*)GPIOI_BASEADDR)

#define RCC         ((RCC_RegDef_t*)RCC_BASEADDR)

#endif //MCU1_STM32F407XX_H
