//
// Created by andy- on 2022-01-13.
//

#ifndef MCU1_FONTS_H
#define MCU1_FONTS_H

#define MAX_HEIGHT_FONT         41
#define MAX_WIDTH_FONT          32
#define OFFSET_BITMAP

#include <cstdint>

// ASCII
typedef struct _tFont {
    const uint8_t *table;
    uint16_t Width;
    uint16_t Height;
} sFONT;

// GB2312
typedef struct {
    unsigned char index[2];
    const char matrix[MAX_HEIGHT_FONT * MAX_WIDTH_FONT / 8];
} CH_CN;

typedef struct {
    const CH_CN *table;
    uint16_t size;
    uint16_t ASCII_Width;
    uint16_t Width;
    uint16_t Height;
} cFONT;

extern sFONT Font24;
extern sFONT Font20;
extern sFONT Font16;
extern sFONT Font12;
extern sFONT Font8;

extern cFONT Font12CN;
extern cFONT Font24CN;

#endif //MCU1_FONTS_H
