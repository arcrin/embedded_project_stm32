#include "stm32f407xx.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

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
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

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
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    // 3. Send the address of the slave with R/nW bit set to W(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in SR1
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    // 5. Clear the ADDR flag according to its software sequence
    // Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle);

    // 6. Send the data until len becomes 0
    while (Len > 0) {
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. when Len becomes zero wait for TXE=1 and BFE=1 before generating the STOP condition
    // NOTE: TXE=1, BTF=1, means that both SR and DR are emtpy and next transmission should begin
    // when BTF=1 SCL will be stretched
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    // 8. Generate STOP condition and master need not wait for the completion of stop condition
    if (Sr == I2C_DISABLE_SR) {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}

void I2C_Master_ReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
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
        I2C_ClearADDRFlag(pI2CHandle);

        // read the data until Len becomes zero

    }
}