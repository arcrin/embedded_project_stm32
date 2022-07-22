//
// Created by wbai on 7/14/2022.
//
#include <cstring>
#include <cstdio>
#include "stm32f407xx.h"

// flg variable
uint8_t rxComplt = RESET;

#define DEV_BOARD_ADDR  0x61
#define SLAVE_ADDR      0x68

I2C_Handle_t I2C1Handle;

// receive buffer
uint8_t rcv_buf[32];

/*
 * PB6-> SCL
 * PB7-> SDA
 */

void I2C1_GPIOInits()
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    //scl
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);


    //sda
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = DEV_BOARD_ADDR;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn,GpioLed;

    //this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);

    //this is led gpio configuration
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD,ENABLE);

    GPIO_Init(&GpioLed);
}

int main(){
    disable_irq();
    SysTick_Init(16000);
    uint8_t commandcode;
    uint8_t len;

    GPIO_ButtonInit();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

    enable_irq();

    while(1){
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay(200);

        commandcode = 0x51;

        while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        commandcode = 0x52;

        while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

        rxComplt = RESET;

        while(rxComplt != SET);

        rcv_buf[len + 1] = '\0';
        rxComplt = RESET;
    }
}

extern "C" {
    void I2C1_EV_IRQHandler(){
        I2C_EV_IRQHandling(&I2C1Handle);
    }
}

extern "C" {
    void I2C1_ER_IRQHandler(){
        I2C_ER_IRQHandling(&I2C1Handle);
    }
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
    if (AppEv == I2C_EV_TX_CMPLT) {
        // printf("Tx is completed")
    } else if (AppEv == I2C_EV_RX_CMPLT) {
        // printf("Rx is completed.\n")
        rxComplt = SET;
    } else if (AppEv == I2C_ERROR_AF) {
        // in master ack failure happens when slave fails to send ack for the byte
        // sent from the master
        I2C_CloseSendData(pI2CHandle);

        // generate the stop condition to release the bus
        I2C_GenerateStopCondition(I2C1);

        // Hand in infinite loop
        while(1);
    }
}