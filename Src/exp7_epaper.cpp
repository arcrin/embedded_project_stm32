//
// Created by wbai on 1/11/2022.
//

#ifndef MCU1_EXP7_EPAPER_CPP
#define MCU1_EXP7_EPAPER_CPP

#include "EPD_2in66.h"
#include "GUI_Paint.h"

int main(){
    if (DEV_Module_Init() != 0) {
        return -1;
    }

    EPD_2IN66_Init();
    EPD_2IN66_Clear();
    DEV_Delay(10);

}

#endif //MCU1_EXP7_EPAPER_CPP
