#include "stm32f407xx.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << 0);
    }
}

void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        if(pI2Cx == I2C1){
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        // TODO: disable I2C clock?
    }
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
    //note: somehow I need to manually clear the stop bit in CR1?!
    while(!(pI2Cx->CR1 & (1 << I2C_CR1_STOP)));
    pI2Cx->CR1 &= ~(1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1); // set R/nW bit to 0, the 8th bit
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1; // set R/nW bit to 1, the 8th bit
    pI2Cx->DR = SlaveAddr;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
    if (EnOrDi == I2C_ACK_ENABLE) {
        // enable the ack
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    } else {
        // disable the ack
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
    uint32_t dummy_read;
    // check for device mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
        // device is in master mode
        if (pI2CHandle->TxRxState == (I2C_BUSY_IN_RX)) {
            if (pI2CHandle->RxSize == 1) {
                // first disable the ack
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                // clear the ADDR flag
                // (ADDR is cleared in software by reading SR1 then reading SR2, or by hardware when PE=0)
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void) dummy_read;
            }
        } else {
            // clear the ADDR flag
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void) dummy_read;
        }
    } else {
        // device is in slave mode
        // clear the ADDR flag
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void) dummy_read;
    }
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
    if (EnOrDi == ENABLE) {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    } else {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
    if (pI2Cx->SR1 & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
    uint32_t tempreg = 0;

    // enable the clock for the i2cx peripheral
    I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);

    // ack control bit
    tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // config the FREQ field of CR2
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

    // program the device own address
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // CCR calculations
    uint16_t ccr_value = 0;
    tempreg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // mode is standard mode
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (ccr_value & 0xfff);
    } else {
        // mode is fast mode
        tempreg |= (1 << 15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        } else {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempreg |= (ccr_value & 0xfff);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

    // TRISE configuration
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // standard mode
        tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
    }else{
        // fast mode
        tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
    }
    pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
 
    // 2. Confirm that start generation is completed by checking the SB flag in the SR1
    // NOTE: until SB is cleared SCL will be stretched (pulled to LOW)
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)); // waiting for this flag to be set

    // 3. Send the address of the slave with R/nW bit set to W(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in SR1
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    // 5. Clear the ADDR flag according to its software sequence
    // Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle);

    // 6. Send the data until len becomes 0
    while (Len > 0) {
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait until data register is empty
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. when Len becomes zero wait for TXE=1 and BFE=1 before generating the STOP condition
    // NOTE: TXE=1, BTF=1, means that both SR and DR are emtpy and next transmission should begin
    // when BTF=1 SCL will be stretched
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); // wait for byte transfer is finished

    // 8. Generate STOP condition and master need not wait for the completion of stop condition
    if (Sr == I2C_DISABLE_SR) {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Confirm that start generation is completed by checking the SB flag in SR1
    // Note: until SB is cleared SCL will be stretched (pulled LOW)
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    // 3. Send address of the slave with R/nW bit et to R(1) (total 8 bits)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. wait until address phase is completed by checking the ADDR flag in SR1
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    // procedure to read only 1 byte from slave
    if (Len == 1) {
        // Disable Acking
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // wait until RXNE becomes 1 (dara register not empty, indicating data was received)
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        if (Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // read data in to buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    // procedure to read data from slave when Len > 1
    if (Len > 1) {
        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle); // data reception will only begin after ADDR flag is cleared

        // read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--) {
            // wait until RXNE becomes 1
            while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
            if (i == 2) { // if last 2 bytes are remaining
                // Disable Acking
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                // Generate STOP condition
                if (Sr == I2C_DISABLE_SR) {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }

            // read the data from data register in to buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            // increment the buffer address
            pRxBuffer++;
        }
    }
    // re-enable ACKing
    if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
    if(EnOrDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ISER0 |= ( 1 << IRQNumber );

        }else if(IRQNumber > 31 && IRQNumber < 64 )
        {
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if(IRQNumber >= 64 && IRQNumber < 96 )
        {
            *NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
        }
    }else
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        }else if(IRQNumber > 31 && IRQNumber < 64 )
        {
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if(IRQNumber >= 6 && IRQNumber < 96 )
        {
            *NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
        }
    }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber %4 ;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NVIC_PRIORITY_BITS) ;

    *(NVIC_IPR_BASEADDR + iprx) |=  (IRQPriority << shift_amount);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
    uint8_t busystate = pI2CHandle->TxRxState; // take a note of the current state
    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){ // if the buss is neither transmitting nor receiving
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX; // set the state to transmit
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // generate start condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // set ITBUFEN bit (buffer interrupt enable, related to TxE and RxNE flag)
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // enable ITEVTEN bit (event interrupt enable
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // enable ITERREN bit (error interrupt enable)
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
    uint8_t busystate = pI2CHandle->TxRxState;
    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) { // if the bus is not busy in transmitting or receiving
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX; // set the state to receive
        pI2CHandle->RxSize = Len;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
        // same control bits are enabled as transmission
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
    if(pI2CHandle->TxLen > 0) {
        // 1. load the data in to DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        // 2. decrement the TxLen
        pI2CHandle->TxLen--;

        // 3. Increment the buffer address
        pI2CHandle->pTxBuffer++;
    }
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
    // We have to do the data reception
    if (pI2CHandle->RxSize == 1) {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1) {
        if (pI2CHandle->RxLen == 2) {
            // clear the ack bit
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }
        // read DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }
    if (pI2CHandle->RxLen == 0) {
        // close the I2C data reception and notify the application
        // 1. generate the stop condition
        if (pI2CHandle->Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }
        // 2. close the I2C rx
        I2C_CloseReceiveData(pI2CHandle);

        // 3. Notify the application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
    // reset ITBUFEN bit (buffer interrupt enable)
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // reset ITEVTEN control bit (event interrupt enable)
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
    // reset ITBUFEN bit (buffer interrupt enable bit)
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // reset ITEVTEN bit (event interrupt enable)
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data){
    pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
    return (uint8_t) pI2C->DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
    // Interrupt handling for both master and slave mode of a device
    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN); // read if event interrupt bit-status
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN); // read if buffer interrupt bit-status
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB); // read SB status flag

    // 1. SB (start bit generated) event handle
    // Note: SB flag is only applicable in master mode
    if(temp1 && temp3){ // event interrupt enabled and start condition generated
        // event generated by start condition generation (SB event)
        // execute address phase in this handle
        // this block will not be executed in slave mode, because SB is always zero
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1; // read ADDR status flag
    temp3 = temp3 & (1 << I2C_SR1_ADDR); // read ADDR status flag

    // 2. ADDR event handle
    // Note: In master mode: Address is sent
    //       In slave mode: Address is matched with own address
    if (temp1 && temp3) {
        I2C_ClearADDRFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    // 3. Handle byte transfer finished event (BTF)
    if (temp1 && temp3) {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) { // if TxE is also set
            // BTF==1, TxE==1
            // a new byte should be sent and DR has not been written
            if (pI2CHandle->TxLen == 0) {
                // 1. generate the STOP condition
                if (pI2CHandle->Sr == I2C_DISABLE_SR) {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
                // 2. reset all the member elements of the handle structure
                I2C_CloseSendData(pI2CHandle);

                // 3. notify the application about transmission complete
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
            }
        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            ;
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    // 4. Handle for interrupt generated by STOPF event (stop condition detection)
    // Note: Stop detection flag is only applicable for slave mode. This flag will never be set for master mode
    // This code block will not be executed by the master, since STOPF will not set in master mode
    if(temp1 && temp3) {
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        // Notify the application that sSTOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    // 5. Handle for interrupt generated by TXE event
    if (temp1 && temp2 && temp3) {
        // Check for device mode
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // TXE flag is set
            // We have to do the data transmission
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        } else {
            // slave mode
            // make sure that the slave is really in tranmitter mode
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    // 6. Handle for interrupt generated by RXNE event
    if (temp1 && temp2 && temp3) { // RxNE is set
        // check device mode
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // master mode
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        } else{
            // slave mode
            // make sure that the slave is actually in receiver mode
            if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
    uint32_t ITERREN_BIT;

    // read the status of ITERREN control bit in the CR2
    ITERREN_BIT = ((pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN)) >> I2C_CR2_ITERREN;
    /***************Check for Bus Error*******************/
    uint32_t BERR_FLAG;
    BERR_FLAG = ((pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR)) >> I2C_SR1_BERR;
    if (BERR_FLAG & ITERREN_BIT) {
        // This is Bus error
        // clear the bus error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        // notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }
    /************Check for arbitration lost error*************/
    uint32_t ARLO_FLAG;
    ARLO_FLAG = ((pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO)) >> I2C_SR1_ARLO;
    if (ARLO_FLAG && ITERREN_BIT) {
        // This is arbitration lost error
        // clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        // notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /*************Check for ACK failure error******************/
    uint32_t AF_FLAG;
    AF_FLAG = ((pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF)) >> I2C_SR1_AF;
    if (AF_FLAG & ITERREN_BIT) {
        // This is ACK failure error
        // clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        // notify the application about error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /*************Check for Overrun/underrun error*************/
    uint32_t OVR_FLAG;
    OVR_FLAG = ((pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR)) >> I2C_SR1_OVR;
    if (OVR_FLAG & ITERREN_BIT) {
        // This is overrun/underrun error
        // clear overrun/underrun flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        // notify application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /**************Check for Time out error*********************/
    uint32_t TIMEOUT_FLAG;
    TIMEOUT_FLAG = ((pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT))>> I2C_SR1_TIMEOUT;
    if (TIMEOUT_FLAG && ITERREN_BIT) {
        // This is Time out error
        // clear the Time out error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        // notify application about this error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}