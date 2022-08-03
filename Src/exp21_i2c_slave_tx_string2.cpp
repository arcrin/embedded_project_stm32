//
// Created by wbai on 8/3/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"


#define my_addr     0x68
#define slave_addr  0x68

uint32_t data_len = 0;


uint8_t tx_buffer[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123A";
//uint8_t tx_buffer[] = {0x0,
//                       0x1,
//                       0x2,
//                       0x3,
//                       0x4,
//                       0x5,
//                       0x6,
//                       0x7,
//                       0x8,
//                       0x9,
//                       0xa,
//                       0xb,
//                       0xc,
//                       0xd,
//                       0xe,
//                       0xf,
//                       0x10,
//                       0x11,
//                       0x12,
//                       0x13,
//                       0x14,
//                       0x15,
//                       0x16,
//                       0x17,
//                       0x18,
//                       0x19,
//                       0x1a,
//                       0x1b,
//                       0x1c,
//                       0x1d,
//};

uint8_t commandCode;

I2C_Handle_t I2C1Handle;

uint8_t receive_buffer[32];

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
    I2C1Handle.I2C_Config.I2C_DeviceAddress = my_addr;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn;

    //this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(){
    data_len = strlen((char *) tx_buffer);
//    data_len = sizeof(tx_buffer);
    GPIO_ButtonInit();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

    while(1);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
    static uint32_t count = 0;
    static uint32_t w_ptr = 0;

    if (AppEv == I2C_ERROR_AF) {
        // This will happen during slave transmitting data to master
        // slave should understand master needs no more data
        // slave concludes end of TX
        if (commandCode != 0x52) {
            commandCode = 0xFF;
        } else {
            if (w_ptr > 0) {
                w_ptr--;
            }
        }
        count = 0;
        if (w_ptr >= (data_len)) {
            w_ptr = 0;
            commandCode = 0xFF;
        }
    } else if (AppEv == I2C_EV_STOP){
        count = 0;
    } else if (AppEv == I2C_EV_DATA_REQ) {
        if (commandCode == 0x51) {
            // Here we are sending 4 bytes of length information
            I2C_SlaveSendData(I2C1, ((data_len >> (count % 2) * 8)) & 0xFF);
            count++;
        } else if (commandCode == 0x52) {
            I2C_SlaveSendData(I2C1, tx_buffer[w_ptr++]);
        }
    } else if (AppEv == I2C_EV_DATA_RCV) {
        //Master has sent command code, read it
        commandCode = I2C_SlaveReceiveData(I2C1);
    }
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
