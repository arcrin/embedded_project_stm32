//
// Created by andy- on 2021-10-26.
//



#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H
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
#define PERIPH_BASE         0x4000000
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     0x40010000
#define AHB1PERIPH_BASE     0x40020000
#define AHB2PERIPH_BASE     0x50000000


/*
 * GPIO ports base addresses
 */
#define GPIOA_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASE + 0x0400)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOa_BASEADDR      (AHB1PERIPH_BASE + 0x0000)

#endif //MCU1_STM32F407XX_H
