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

/*
 * SPI control register_1 bit definition macros
 */
#define SPI_CR1_CPHA        0 // clock phase
#define SPI_CR1_CPOL        1 // clock polarity
#define SPI_CR1_MSTR        2 // master selection
#define SPI_CR1_BR          3 // [2:0] baud rate
#define SPI_CR1_SPE         6 // SPI enable
#define SPI_CR1_LSBFIRST    7 // frame format MSB or LSB first
#define SPI_CR1_SSI         8 // internall slave select, force its valie on NSS in software slave mode
#define SPI_CR1_SSM         9 // software slabe managemet
#define SPI_CR1_RXONLY      10 // receive only
#define SPI_CR1_DFF         11 //data frame format (8-bit of 16-bit)
#define SPI_CR1_CRCNEXT     12 // CRC transfer next
#define SPI_CR1_CRCEN       13 // hardware CRC calculation enable
#define SPI_CR1_BIDIOE      14 // output enable in bidirectional mode
#define SPI_CR1_BIDI        15 // bidirectional data mode enable

/*
 * SPI control register_2 bit definition macros
 */
#define SPI_CR2_RXDMAEN     0 // rx buffer DMA(direct memory access) enable
#define SPI_CR2_TXDMAEN     1 // tx buffer DMA enable
#define SPI_CR2_SSOE        2 // SS ourput enable TODO: ?
#define SPI_CR2_FRF         4 // frame format
#define SPI_CR2_EERIE       5 // error interrupt enable
#define SPI_CR2_RXNEIE      6 // rx_buffer_not_empty_interrupt enable
#define SPI_CR2_TXEIE       7 // tx_buffer_empty_interrupt enable

/*
 * SPI status register bit definition macros
 */
#define SPI_SR_RXNE         0 // rx buffer not empty
#define SPI_SR_TXE          1 // tx buffer empty
#define SPI_SR_CHSIDE       2 // channel side TODO: ?
#define SPI_SR_UDR          3 // underrun flag TODO: ?
#define SPI_SR_CRCEER       4 // CRC error flag
#define SPI_SR_MODF         5 // mode fault TODO: ?
#define SPI_SR_OVR          6 // overrun flag TODO: ?
#define SPI_SR_BUSY          7 // busy flag
#define SPI_SR_FRE          8 // frame format error

/*
 * SPI flag definition macros
 */
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG        (1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG       (1 << SPI_SR_BUSY)


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

/*
 * other peripheral APIs
 */
void SPI_PeriControl(pSPI_RegDef_t pSPIx, uint8_t EnOrDi);

void SPI_SSIConfig(pSPI_RegDef_t pSPIx, uint8_t EnOrDi);

void SPI_SSOEConfig(pSPI_RegDef_t pSPIx, uint8_t EnOrDi);

uint8_t SPI_GetFlagStatus(pSPI_RegDef_t pSPIx, uint32_t FlagName);

#endif //MCU1_STM32F407_SPI_DRIVER_H
