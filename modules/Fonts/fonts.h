//
// Created by wbai on 5/19/2022.
//

#ifndef MCU1_FONTS_H
#define MCU1_FONTS_H
#ifdef __cplusplus
extern "C" {
#endif
#include <cstdint>
//AsCII
typedef struct _tFont{
    const uint8_t *table;
    uint16_t Width;
    uint16_t Height;
}sFONT;

extern sFONT Font24;
extern sFONT Font20;
extern sFONT Font16;
extern sFONT Font12;
extern sFONT Font8;
#ifdef __cplusplus
}
#endif

#endif //MCU1_FONTS_H
