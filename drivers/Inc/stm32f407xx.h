//
// Created by andy- on 2021-10-26.
//

#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM1_SIZE          112 * 1024
#define SRAM2               SRAM1_BASEADDR + SRAM1_SIZE // 0X2001C000
#define ROM
#define SRAM                SRAM1_BASEADDR

#endif //MCU1_STM32F407XX_H
