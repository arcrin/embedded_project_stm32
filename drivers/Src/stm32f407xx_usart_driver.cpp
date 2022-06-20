//
// Created by wbai on 6/17/2022.
//
#include "stm32f407xx_usart_driver.h"



/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        if (pUSARTx == USART1) {
            USART1_PCLK_EN();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_EN();
        } else if (pUSARTx == USART3) {
            USART3_PCLK_EN();
        } else if (pUSARTx == UART4) {
            UART4_PCLK_EN();
        } else if (pUSARTx == UART5) {
            UART5_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_EN();
        }
    } else {
        if (pUSARTx == USART1) {
            USART1_PCLK_DI();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_DI();
        } else if (pUSARTx == USART3) {
            USART3_PCLK_DI();
        } else if (pUSARTx == UART4) {
            UART4_PCLK_DI();
        } else if (pUSARTx == UART5) {
            UART5_PCLK_DI();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_DI();
        }
    }
}
/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle){
    uint32_t tempreg = 0;
/******************************CR1 configuration****************************************************/
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if (pUSARTHandle->USART_Config.USART_Mode == USART_RX_ONLY_MODE) {
        // enable the Receiver bit field
        tempreg |= (1 << USART_CR1_RE);
    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_TX_ONLY_MODE) {
        // enable the Transmitter bit field
        tempreg |= (1 << USART_CR1_TE);
    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_TXRX_MODE) {
        // enable both Receiver and Transmitter bit fields
        tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }
    // configure the Word Length
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    // configure the parity control bit fields
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
        // enable parity control
        tempreg |= (1 << USART_CR1_PCE);
        // Even parity is the default configuration
    } else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
        tempreg |= (1 << USART_CR1_PCE);
        // select odd parity
        tempreg |= (1 << USART_CR1_PS);
    }
    // program CR1
    pUSARTHandle->pUSARTx->CR1 = tempreg;

/***************************************CR2 Configuration*********************************/
    tempreg = 0;
    // stop bit configuration
    tempreg |= pUSARTHandle->USART_Config.USART_NumOfStopBits << USART_CR2_STOP;

    // program CR2 register
    pUSARTHandle->pUSARTx->CR2 = tempreg;
/***************************************CR2 Configuration*********************************/
    tempreg = 0;
    // configure hardware flow control
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
        tempreg |= (1 << USART_CR3_CTSE);
    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
        tempreg |= (1 << USART_CR3_RTSE);
    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
        tempreg |= (1 << USART_CR3_CTSE);
        tempreg |= (1 << USART_CR3_RTSE);
    }
    pUSARTHandle->pUSARTx->CR3 = tempreg;
/******************************** Configuration of BRR(Baudrate register)******************************************/
    // configure baud rate
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
    uint16_t *pdata;
    // Loop over until "Len" number of bytes are transferred
    for (uint32_t i = 0; i < Len; i++) {
        // Implement the code to wait until TXE flag is set in the SR
        while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            // if 9BIT load the DR with 2 bytes masking the bits other than first 9 bits
            pdata = (uint16_t *) pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01ff);

            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                pTxBuffer++;
                pTxBuffer++;
            } else {
                pTxBuffer++;
            }
        } else {
            // This is 8 bit data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);
            // Implement the code to increment the buffer address
            pTxBuffer++;
        }
    }
    while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
    for (uint32_t i = 0; i < Len; i++) {
        while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *((uint16_t *) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01ff);

                pRxBuffer++;
                pRxBuffer++;
            }else{
                *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xff);
                pRxBuffer++;
            }
        } else{
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xff);
            }else{
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7f);
            }
            pRxBuffer++;
        }
    }
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
    uint8_t txstate = pUSARTHandle->TxBusyState;
    if (txstate != USART_BUSY_IN_TX) {
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }
    return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
    uint8_t rxstate = pUSARTHandle->RxBusyState;

    if (rxstate != USART_BUSY_IN_RX) {
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
        (void) pUSARTHandle->pUSARTx->DR;

        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }
    return rxstate;
}

/*
 * IEQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) {
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        } else if (IRQNumber > 64 && IRQNumber < 96) {
            *NVIC_ISER3 |= (1 << IRQNumber % 64);
        }
    } else{
        if (IRQNumber <= 31) {
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) {
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        } else if (IRQNumber > 64 && IRQNumber < 96) {
            *NVIC_ICER3 |= (1 << IRQNumber % 64);
        }
    }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NVIC_PRIORITY_BITS);
    *(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){
    if (pUSARTx->SR & StatusFlagName) {
        return SET;
    }
    return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){
    pUSARTx->SR &= ~(StatusFlagName);
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        pUSARTx->CR1 |= (1 << 13);
    } else {
        pUSARTx->CR1 &= ~(1 << 13);
    }
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
    uint32_t PCLKx;
    uint32_t usartdiv;
    uint32_t M_part, F_part;
    uint32_t tempreg = 0;

    // Get the value of APB bus clock into the variable PCLKx
    if (pUSARTx == USART1 || pUSARTx == USART6) {
        // USART1 and USART6 are hanging on APB2 bus
        PCLKx = RCC_GetPCLK2Value();
    } else{
        PCLKx = RCC_GetPCLK1Value();
    }

    // check for OVER8 configuration bit (over sampling)
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
        // OVER8 = 1, over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    } else {
        // over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    // Calculate the Mantissa part
    M_part = usartdiv / 100;

    // Place the Mantissa part in appropriate bit position. Refer USART_BRR
    tempreg |= M_part << 4;

    // Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    // Calculate the final fractional
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
        // OVER8 = 1, over sampling by 8
        F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07);
    }else {
        // over sampling by 16
        F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0f);
    }
    // Place the fractional part in appropriate bit position. Refer USART_BRR
    tempreg |= F_part;

    // copy the value of tempreg in to BRR register
    pUSARTx->BRR = tempreg;

}
