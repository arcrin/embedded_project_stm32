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
    // device mode
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
    // bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == 1 || pSPIHandle->SPIConfig.SPI_BusConfig == 3){
        pSPIHandle->pSPIx->CR1 |= (0 << 15);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == 2) {
        pSPIHandle->pSPIx->CR1 |= (1 << 15);
    }
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
void SPI_Send_Data(pSPI_RegDef_t pSPIx, uint8_t *pTxBuffer, uint32_t Len); //Standard practice to set length as 32 bit
void SPI_Received_Data(pSPI_RegDef_t pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITConfit(uint8_t IRQNumber, uint8_t ENorDI);

void SPI_IRQPriorityConfig(uint8_t IEQNumber, uint8_t Priority);

void SPI_IRQHandling(pSPI_Handle_t pSpiHandle);
