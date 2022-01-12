//
// Created by wbai on 1/5/2022.
//
#include "stm32f407xx_spi_driver.h"

/****************************************************************
 * APIs supported by SPI driver
 ****************************************************************/
/*
 * Peripheral Clock Setup
 */
/********************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- Initialize or de-initialize peripheral clock for given SPI peripheral
 *
 * @param[pGPIO_RegDef_t]
 *                  - pointer to a SPI peripheral, for stm32f407, SPI1, SPI2, SPI3 are available
 * @param[uint8_t]  - ENABLE or DISABLE
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void SPI_PeriClockControl(pSPI_RegDef_t pSPIx, uint8_t ENorDI){
    if(ENorDI == ENABLE){
        if(pSPIx == SPI1){
            SPI1_PCLK_EN();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_EN();
        }
    } else if (ENorDI == DISABLE) {
        if(pSPIx == SPI1){
            SPI1_PCLK_DI();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */
/********************************************************
 * @fn				- SPI_Init
 *
 * @brief			- Initialize or Deinitialize peripheral clock for given SPI peripheral
 *
 * @param[pGPIO_RegDef_t]
 *                  - pointer to a SPI peripheral, for stm32f407, SPI1, SPI2, SPI3 are available
 * @param[uint8_t]  - ENABLE or DISABLE
 *
 * @return			- None
 *
 * @Note			- None
 ********************************************************/
void SPI_Init(pSPI_Handle_t pSPIHandle){
    // enable clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // device mode
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
    // bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
        // clear BIDI bit
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        // set BIDI bit
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDI);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY) {
        // clear BIDI bit
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI);
        // set RXONLY bit
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);
    }
    // speed config (baud rate)
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
    // data frame format config
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
    // clock polarity config
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
    // clock phase config
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
    // software slave management
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
}

void SPI_DeInit(pSPI_RegDef_t pSPIx){
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }
}

/*
 * Data send and receive
 */
uint8_t SPI_GetFlagStatus(pSPI_RegDef_t pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/********************************************************
 * @fn				- SPI_Send_Data
 *
 * @brief			- blocking api for sending data
 *
 * @param[pSPI_RegDef_t]
 *                  - pointer to SPI peripheral
 * @param[uint8_t]  - pointer to transmit buffer
 * @param[uint23_t] - length of the data to be sent (number of bytes)
 *
 * @return			- None
 *
 * @Note			- blocking call
 ********************************************************/
void SPI_Send_Data(pSPI_RegDef_t pSPIx, uint8_t *pTxBuffer, uint32_t Len) //Standard practice to set length as 32 bit
{
    while (Len > 0) {
        // 1. wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
        // 2. check the DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16-bit DFF
            // 1. load the data to the DR register
            pSPIx->DR = *((uint16_t *) pTxBuffer); // cast to 16-bit pointer first
            Len--;
            Len--;
            (uint16_t *) pTxBuffer++; // same idea here, cast to 16-bit in order to increment the address by 2
        } else {
            // 8-bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}
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
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
    // pSPIx->CR1 |= EnOrDi << SPI_CR1_SPE; // this doesn't clear the bit
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else if (EnOrDi == DISABLE) {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSIConfig(pSPI_RegDef_t pSPIx, uint8_t EnOrDi){
    // the SSI bit works with SSM
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else if (EnOrDi == DISABLE) {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}