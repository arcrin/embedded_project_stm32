//
// Created by wbai on 7/12/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"


#define DEV_BOARD_ADDR  0x61
#define SLAVE_ADDR      0x68

I2C_Handle_t I2C1Handle;

uint8_t rcv_buf[32];

/*
 * PB6 -> SCL
 * PB7(or PB9) -> SDA
 */

void I2C1_GPIOInits(){
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // scl
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // sda
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2CPins);
}

void I2C1_Inits(){
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = DEV_BOARD_ADDR;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(){
    GPIO_Handle_t GPIOBtn;

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(){
    disable_irq();
    SysTick_Init(16000);
    uint8_t command_code;
    uint8_t len;

    GPIO_ButtonInit();

    // i2c pin inits
    I2C1_GPIOInits();

    // enable the i2c peripheral
    I2C1_Inits();

    // enable the i2c peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // ack bit is made 1 after PE=1
    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

    enable_irq();

    while(1){
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(200);
        command_code = 0x51;
        I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);
        command_code = 0x52;
        I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);
        rcv_buf[len + 1] = '\0';
    }
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){}