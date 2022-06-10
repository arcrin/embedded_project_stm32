#include <cstring>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

// command codes
#define COMMAND_LED_CTRL    0x50
#define COMMAND_SENSOR_READ 0x51
#define COMMAND_LED_READ    0x52
#define COMMAND_PRINT       0x53
#define COMMAND_ID_READ     0x54

#define LED_ON  1
#define LED_OFF 0

// arduino analog pins
#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4

// arduino led
#define LED_PIN     9

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */

void SPI2_EPAPER_GPIOInits(void){
    GPIO_Handle_t SPI_Pins;
    SPI_Pins.pGPIOx = GPIOB;
    SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFn_MODE;
    SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPI_Pins.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OP_TYPE_PP;
    SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // MISO pin setup
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPI_Pins); // takes pointer as argument

    // MOSI pin setup
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI_Pins);

    // SCLK pin setup
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI_Pins);

    // NSS pin setup
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPI_Pins);
}

void SPI2_Init(void){
    SPI_Handle_t SPI2_handle;

    SPI2_handle.pSPIx = SPI2;
    SPI2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    SPI2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
}

void GPIO_Button_Init(void){
    GPIO_Handle_t ld4_gpio_handle, b1_gpio_handle;
    ld4_gpio_handle.pGPIOx = GPIOD;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT_MODE;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    ld4_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOD, ENABLE); // this needs to be called before GPIO_Init
    GPIO_Init(&ld4_gpio_handle);

    b1_gpio_handle.pGPIOx = GPIOA;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&b1_gpio_handle);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
    if (ackbyte == (uint8_t) 0xF5) {
        // ack
        return 1;
    }
    return 0;
}

int main(void){
    uint8_t dummy_write = 0xff;
    uint8_t dummy_read;

    GPIO_Button_Init();

    SPI2_GPIOInits();

    SPI_SSOEConfig(SPI2, ENABLE);

    while(1){
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        delay();

        SPI_PeriControl(SPI2, ENABLE);

        // 1. CMD_LED_CTRL <pin no (1)> <value(1)>
        uint8_t command_code = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];

        // send command
        SPI_Send_Data(SPI2, &command_code, 1);

        // do dummy read to clear off the RXNE
        SPI_Receive_Data(SPI2, &dummy_read, 1);

        // Send some dummy bits (1 byte) fetch the response from the slave
        SPI_Send_Data(SPI2, &dummy_write, 1);

        // read the ack byte received
        SPI_Receive_Data(SPI2, &ackbyte, 1);

        if(SPI_VerifyResponse(ackbyte)){
            args[0] = ANALOG_PIN0;

            // send arguments
            SPI_Send_Data(SPI2, args, 1); // sending one byte of

            // do dummy read to clear off the RXNE
            SPI_Receive_Data(SPI2, &dummy_read, 1);

            // insert some delay so that can ready with the data
            delay();

            // send some dummy bits (1 byte) fetch the response from the slave
            SPI_Send_Data(SPI2, &dummy_write, 1);

            uint8_t analog_read;
            SPI_Receive_Data(SPI2, &analog_read, 1);
        }

        // 3. CMD_LED_READ

        // wait till button is pressed
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        // to avoid button de-bouncing related issues, 200ms delay
        delay();

        command_code = COMMAND_LED_READ;

        // send command
        SPI_Send_Data(SPI2, &command_code, 1);

        // do dummy read to clear off the RXNE
        SPI_Receive_Data(SPI2, &dummy_read, 1);

        if(SPI_VerifyResponse(ackbyte)){
            args[0] = LED_PIN;

            // send arguments
            SPI_Send_Data(SPI2, args, 1); // sending one byte

            // do dummy read to clear off the RXNE
            SPI_Receive_Data(SPI2, &dummy_read, 1);

            // insert some delay so that slave can ready with the data
            delay();

            SPI_Send_Data(SPI2, &dummy_write, 1);

            uint8_t led_status;
            SPI_Receive_Data(SPI2, &led_status, 1);
        }

        // 4. CMD_PRINT     <len(2)> <message(len)>
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        // to avoid button de-bouncing
    }
}