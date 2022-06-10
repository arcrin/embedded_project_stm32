//
// Created by wbai on 5/10/2022.
//

#include "epaper.h"
#include "GUI_Paint.h"


extern const unsigned char gImage_100X50[];
extern const unsigned char gImage_2in66[];

void GPIO_Button_Init(){
    GPIO_Handle_t b1_gpio_handle;
    b1_gpio_handle.pGPIOx = GPIOA;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_IN_MODE;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    b1_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&b1_gpio_handle);
}

void SPI2_EPAPER_GPIOInits(){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFN_MODE;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // TODO: why push pull
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // MED or SLOW speed would also work, not sure about VHIGH

//     NSS configure
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

    // SCK configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MISO configuration
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_Init(&SPIPins);

    // MOSI configuration
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);
}

void SPI2_EPAPER_Inits(){
    SPI_Handle_t spi2_handle;
    spi2_handle.pSPIx = SPI2;
    spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    spi2_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
    spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin
    SPI_Init(&spi2_handle);
}

int main(){
    SysTick_Init(16000);
//    SPI2_GPIOInits();
//    SPI2_Inits();
    EPD_GPIO_Init();
    EPD_SPI2_Init();
    GPIO_Button_Init();
    SPI_SSOEConfig(SPI2, ENABLE);

    while(1) {
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        if(DEV_Module_Init() != 0){
            continue;
        }
        EPD_2IN66_Init();
        EPD_2IN66_Clear();
        delay(5000);

        //Create a new image cache
        uint8_t *BlackImage;
        uint16_t Imagesize =
                ((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1)) * EPD_2IN66_HEIGHT;
        if ((BlackImage = (uint8_t *) malloc(Imagesize)) == NULL) {
            return -1;
        }
        Paint_NewImage(BlackImage, EPD_2IN66_WIDTH, EPD_2IN66_HEIGHT, 270, WHITE);

#if 0
        //show image for array
        Paint_SelectImage(BlackImage);
        Paint_Clear(BLACK);
        Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
        Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
        Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);

        Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

        Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawRectangle(80, 70, 130, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

        Paint_DrawCircle(45, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawCircle(105, 95, 20, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);

        Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
        Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

        Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);

        Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);
        Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);

        EPD_2IN66_Display(BlackImage);
#endif

# if 0
        Paint_SelectImage(BlackImage);
        Paint_Clear(WHITE);
        Paint_DrawBitMap(gImage_2in66);
        Paint_DrawBitMap_Paste(gImage_100X50, 10, 10, 100, 50, TRUE);
        EPD_2IN66_Display(BlackImage);
        delay(2000);
#endif



#if 0
        EPD_2IN66_Init_Partial();
        Paint_SelectImage(BlackImage);

        PAINT_TIME sPaint_time;
        sPaint_time.Hour = 12;
        sPaint_time.Min = 34;
        sPaint_time.Sec = 56;
        uint16_t num = 10;

        for (;;){
            sPaint_time.Sec = sPaint_time.Sec + 1;
            if (sPaint_time.Sec == 60) {
                sPaint_time.Min = sPaint_time.Min + 1;
                sPaint_time.Sec = 0;
                if (sPaint_time.Min == 60) {
                    sPaint_time.Hour = sPaint_time.Hour + 1;
                    sPaint_time.Min = 0;
                    if (sPaint_time.Hour == 24) {
                        sPaint_time.Hour = 0;
                        sPaint_time.Min = 0;
                        sPaint_time.Sec = 0;
                    }
                }
            }
            Paint_ClearWindows(180, 100, 296, 152, WHITE);
            Paint_DrawTime(180, 110, &sPaint_time, &Font20, WHITE, BLACK);

            num = num - 1;
            if (num == 0) {
                break;
            }
            EPD_2IN66_Display(BlackImage);
            delay(500);
        }
        EPD_2IN66_Clear();
#endif

#if 1
        Paint_SelectImage(BlackImage);
        Paint_Clear(WHITE);
        Paint_DrawString_EN(10, 20, "hello world", &Font24, BLACK, WHITE);
        delay(200);
        EPD_2IN66_Display(BlackImage);
#endif


        //*******************
//        EPD_2IN66_Init();
//        EPD_2IN66_Clear();
//        delay(5000);
        delay(3000);
        EPD_2IN66_Sleep();
        free(BlackImage);
        BlackImage = NULL;

        DEV_Module_Exit();
    }
}
