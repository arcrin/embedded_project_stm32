//
// Created by wbai on 1/5/2022.
//
#include <cstdint>
//#ifdef MCU1_STM32F407XX_H
#include "stm32f407xx.h"
//#endif

#ifndef MCU1_STM32F407_SPI_DRIVER_H
#define MCU1_STM32F407_SPI_DRIVER_H

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
    pSPI_RegDef_t   pSPIx;
    SPI_Config_t SPIConfig;
}SPI_Handle_t, *pSPI_Handle_t;

/*
 * @SPI_DeviceMode
 * BIDIMODE bit:
 * 0: 2-line unidirectional data mode, full duplex, combine with RXONLY bit
 * 1: 1-line bidirectional data mode, half duplex, combine with BIDIOE bit
 */
#define SPI_DEVICE_MODE_SLAVE   0
#define SPI_DEVICE_MODE_MASTER  1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD            1
#define SPI_BUS_CONFIG_HD            2
#define SPI_BUS_CONFIG_SIMP_RXONLY   3
//#define SPI_BUS_CONFIG_SIMP_TXONLY   this is just full duplex with RX pin (MISO) removed/ignored

/*
 * @SPI_SclkSpeed (serial clock speed)
 */
#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

/*
 * @SPI_DFF (data frame format)
 */
#define SPI_DFF_8BITS      0
#define SPI_DFF_16BITS     1

/*
 * @SPI_CPOL (clock polarity, decides the idle state)
 */
#define SPI_CPOL_LOW    0
#define SPI_CPLO_HITH   1

/*
 * @SPI_CPHA (clock phase, decides the data capture transition)
 */
#define SPI_CPHA_LOW    0
#define SPI_CPHA_HIGH   1

/*
 * @SPI_SSM (software slave management, when SSM is set, NSS pin input is replaced
 * with value from the SSI bit)
 */
#define SPI_SSM_DI      0
#define SPI_SSM_EN      1

/****************************************************************
 * APIs supported by SPI driver
 ****************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(pSPI_RegDef_t pSPIx, uint8_t ENorDI);

/*
 * Init and De-init
 */
void SPI_Init(pSPI_Handle_t pSPIHandle);

void SPI_DeInit(pSPI_RegDef_t pSPIx);

/*
 * Data send and receive
 */
void SPI_Send_Data(pSPI_RegDef_t pSPIx, uint8_t *pTxBuffer, uint32_t Len); //Standard practice to set length as 32 bit
void SPI_Received_Data(pSPI_RegDef_t pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITConfit(uint8_t IRQNumber, uint8_t ENorDI);

void SPI_IRQPriorityConfig(uint8_t IEQNumber, uint8_t Priority);

void SPI_IRQHandling(pSPI_Handle_t pSpiHandle);


#endif //MCU1_STM32F407_SPI_DRIVER_H
