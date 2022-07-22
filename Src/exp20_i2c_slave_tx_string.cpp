//
// Created by wbai on 7/18/2022.
//
#include <cstdio>
#include <cstring>
#include "stm32f407xx.h"

#define SLAVE_ADDR      0x69
#define DEV_BOARD_ADDR  SLAVE_ADDR

I2C_Handle_t I2C1Handle;

uint8_t rx_buf[32] = "STM32 Slave mode testing";


/*
 * PB6 -> SCL
 * PB7 -> SDA
 */
void I2C1_GPIOInits(){
    GPIO_Handle_t I2C1Pins;
    I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2C1Pins.GPIO_PinConfig.GPIO_PinMode = 4;
    I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    //scl
    I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2C1Pins);

    //sda
    I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2C1Pins);
}