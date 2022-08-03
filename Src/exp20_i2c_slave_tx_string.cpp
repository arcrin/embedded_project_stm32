//
// Created by andy- on 2022-08-01.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"


#define slave_addr  0x69
#define my_addr     slave_addr

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t tx_buffer[32] = "STM32 Slave mode testing\n";

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */
void I2C1_GPIOInits(){
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

    //scl
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    //sda
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2CPins);
}

void I2C1_Inits(){
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = my_addr;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(){
    GPIO_Handle_t GPIOBtn;

    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(){
    GPIO_ButtonInit();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

    while(1);
}

extern "C"{
    void I2C1_EV_IRQHandler(){
        I2C_EV_IRQHandling(&I2C1Handle);
    }
}

extern "C"{
    void I2C1_ER_IRQHandler(){
        I2C_ER_IRQHandling(&I2C1Handle);
    }
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
    static uint8_t commandCode = 0;
    static uint8_t count = 0;

    if (AppEv == I2C_EV_DATA_REQ) {
        if (commandCode == 0x51) {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char *) tx_buffer));
        } else if(commandCode == 0x52) {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buffer[count++]);
        }
    } else if (AppEv == I2C_EV_DATA_RCV) {
        commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
    } else if (AppEv == I2C_ERROR_AF) {
        commandCode == 0xff;
        count = 0;
    } else if (AppEv == I2C_EV_STOP) {
        //This only happens during slave reception.
        //Master has ended the I2C communication with the slave.
    }
}