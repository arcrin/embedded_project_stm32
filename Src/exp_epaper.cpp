//
// Created by wbai on 5/10/2022.
//

#include "epaper.h"
#include "GUI_Paint.h"
#include "barcode.h"
#include <numeric>


extern const unsigned char gImage_100X50[];
extern const unsigned char gImage_2in66[];
extern const unsigned char tag_logo_24x24[];
extern const unsigned char tag_logo_100x100[];
extern const unsigned char label_image[];

enum code128b{
};
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
        uint8_t *bw_image;
        uint16_t image_size =
                ((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1)) * EPD_2IN66_HEIGHT;
        if ((bw_image = (uint8_t *) malloc(image_size)) == NULL) {
            return -1;
        }
        Paint_NewImage(bw_image, EPD_2IN66_WIDTH, EPD_2IN66_HEIGHT, ROTATE_270, WHITE);

#if 0
        //show image for array
        Paint_SelectImage(bw_image);
        Paint_Clear(WHITE);
//        bw_image[0] = 0x3f;
//        bw_image[19] = 0x3f;
//        bw_image[37] = 0x0;
//        bw_image[74] = 0x0;
//        bw_image[111] = 0x0;
//        Paint_DrawPoint(50, 1, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
//        Paint_DrawPoint(10, 80, WHITE, DOT_PIXEL_1X1, DOT_STYLE_DFT);
//        Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
//        Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
////
//        Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
//        Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
//
//        Paint_DrawRectangle(20, 70, 20, 100, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
//        Paint_DrawRectangle(30, 70, 30, 100, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
//        Paint_DrawRectangle(80, 70, 81, 71, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
//
//        Paint_DrawCircle(45, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
//        Paint_DrawCircle(105, 95, 20, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
//
//        Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
//        Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
//
        Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);

//        Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);
//        Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);

        EPD_2IN66_Display(bw_image);
#endif

# if 0
        Paint_SelectImage(bw_image);
        Paint_Clear(WHITE);
//        Paint_DrawBitMap(gImage_2in66);
//        Paint_DrawBitMap_Paste(gImage_100X50, 10, 10, 100, 50, TRUE);
//        Paint_DrawBitMap_Paste(tag_logo_100x100, 1, 1, 100, 100, FALSE);
        Paint_DrawBitMap_Paste(tag_logo_24x24, 1, 1, 24, 24, FALSE);
        EPD_2IN66_Display(bw_image);
        delay(2000);
#endif



#if 0
        EPD_2IN66_Init_Partial();
        Paint_SelectImage(bw_image);
        Paint_Clear(WHITE);
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
//            Paint_Clear(WHITE);
            Paint_DrawTime(180, 110, &sPaint_time, &Font20, BLACK, WHITE);

            num = num - 1;
            if (num == 0) {
                break;
            }
            EPD_2IN66_Display(bw_image);
            delay(1000);
        }
        EPD_2IN66_Clear();
#endif

#if 0
        Paint_SelectImage(BlackImage);
        Paint_Clear(WHITE);
        Paint_DrawString_EN(10, 20, "hello world", &Font24, BLACK, WHITE);
        delay(200);
        EPD_2IN66_Display(BlackImage);
#endif

#if 1
        char product_firmware_label[] = "Firmware: 4.11.293";
        char jig_revision_label[] = "Jig Rev: 0.0";
        char product_name_label[] = "Product: O3-DIN-Modules";
        char instrument_id[] = "PD-396";
        Paint_SelectImage(bw_image);
        Paint_Clear(WHITE);


        std::vector<int> bar_widths = barcode_widths(std::string(instrument_id), 2);
        uint16_t total_bar_code_width = 0;
        total_bar_code_width = std::accumulate(bar_widths.begin(), bar_widths.end(), decltype(bar_widths)::value_type(0));
        Paint_DrawString_EN(0, 0, product_name_label, &Font16, BLACK, WHITE);
        Paint_DrawString_EN(0, 17, product_firmware_label, &Font16, BLACK, WHITE);

        int bar_code_start_position = (Paint.Width - total_bar_code_width) / 2;
        uint instrument_id_start_position =
                bar_code_start_position + (total_bar_code_width - std::string(instrument_id).size() * Font16.Width) / 2;
        uint jig_rev_start_position =
                bar_code_start_position + (total_bar_code_width - std::string(jig_revision_label).size() * Font16.Width) / 2;
        for (uint i = 0; i < bar_widths.size(); i++) {
            for (int j = 0; j < bar_widths[i]; j++) {
                if (i % 2 == 0) {
                    Paint_DrawLine(bar_code_start_position, 51, bar_code_start_position, 103, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
                } else{
                    Paint_DrawLine(bar_code_start_position, 51, bar_code_start_position, 103, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
                }
                bar_code_start_position += 1;
            }
        }
        Paint_DrawString_EN(instrument_id_start_position, 104, instrument_id, &Font16, BLACK, WHITE);
        Paint_DrawString_EN(jig_rev_start_position, 120, jig_revision_label, &Font16, BLACK, WHITE);
        Paint_DrawBitMap_Paste(tag_logo_24x24, 0, 127, 24, 24, FALSE);

        char tag_message[] = "TestandAutomation@deltacontrols.com";
        uint tag_message_start_position_x = (Paint.Width - strlen(tag_message) * Font12.Width) / 2;

        delay(200);
        EPD_2IN66_Display(bw_image);
#endif
#if 1
        EPD_2IN66_Display((uint8_t*)&label_image);
#endif


        //*******************
//        EPD_2IN66_Init();
//        EPD_2IN66_Clear();
//        delay(5000);
        delay(3000);
        EPD_2IN66_Sleep();
        free(bw_image);
        bw_image = NULL;

//        DEV_Module_Exit(); // calling this function seems to kill some pixels
    }
}
