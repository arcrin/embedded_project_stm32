//
// Created by wbai on 1/5/2022.
//
#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
void SPI_Received_Data(pSPI_RegDef_t pSPIx, uint8_t *pRxBuffer, uint32_t Len){
    while(Len > 0){
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            *(uint16_t *) pRxBuffer = pSPIx->DR;
            Len--;
            Len--;
            (uint16_t *) pRxBuffer++;
        } else {
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t ENorDI){
    if (ENorDI == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 |= (0x1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber <= 63) {
            *NVIC_ISER1 |= (0x1 << IRQNumber % 32);
        } else if (IRQNumber > 63 && IRQNumber <= 96) {
            *NVIC_ISER2 |= (0x1 << (IRQNumber % 64));
        }
    } else {
        if (IRQNumber <= 31) {
            *NVIC_ICER0 |= (0x1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber <= 63) {
            *NVIC_ICER1 |= (0x1 << IRQNumber % 32);
        } else if (IRQNumber > 63 && IRQNumber <= 96) {
            *NVIC_ICER2 |= (0x1 << (IRQNumber % 64));
        }
    }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority){
    uint8_t ipr_register_offset = IRQNumber / 4;
    uint8_t ipr_byte_offset = IRQNumber % 4;
    *(NVIC_IPR_BASEADDR + (0x4 * ipr_register_offset)) |= ((Priority << NVIC_PRIORITY_BITS) << (ipr_byte_offset * 8));
}

void SPI_IRQHandling(pSPI_Handle_t pSPIHandle){
    uint8_t temp1, temp2;
    // first lets check for TXE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    if (temp1 && temp2) {
        // handle TXE
        spi_txe_interrupt_handle(pSPIHandle);
    }

    // check for RXNE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2) {
        // handle RXNE
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    // check for ovr flag
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    if (temp1 && temp2) {
        // handle ovr error
        spi_ovr_err_interrupt_handle(pSPIHandle);
    }
}

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

/**********************************************************
 * works with hardware slave management
 **********************************************************/
void SPI_SSOEConfig(pSPI_RegDef_t pSPIx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else if (EnOrDi == DISABLE) {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}


uint8_t SPI_Send_Data_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
    uint8_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX) {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        // 2. Mark the SPI state as busy in transmission so that
        // no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
        // 4. Data Transmission will be handled by the ISR code ( will implement later )
    }
    return state;
}

uint8_t SPI_Receive_Data_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
    uint8_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX) {
        // 1. Save the Rx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        // 2. Mark the SPI state as busy in reception so that
        // no other code can take over same SPI peripheral until reception is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;
        // 3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }
    return state;
}

// helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
    // check the DFF bit in CR1
    if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
        // 16 bit DFF
        // 1. load the data in to DR
        pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t *) pSPIHandle->pTxBuffer++;
    } else {
        // 8 bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }
    if (!pSPIHandle->TxLen) {
        // TxLen is zero, so close the spi transmission and inform the application that
        // TX is over

        // this prevents interrupts from setting up of TXE flag
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
    // do rxing as per the DFF
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16 bit
        *((uint16_t *) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer++;
        pSPIHandle->pRxBuffer++;
    } else {
        // 8 bit
        *(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }
    if(!pSPIHandle->RxLen) {
        // reception is complete
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
    uint8_t temp;
    // 1. clear the ovr flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void) temp;
    // 2. inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
    uint8_t temp;
    // clearing the OVR bit is done by a read from the SPI_DR register followed by
    // a read access to the SPI_SR register
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void) temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEV){
    // This is weak implementation. The user application may override this function.
}