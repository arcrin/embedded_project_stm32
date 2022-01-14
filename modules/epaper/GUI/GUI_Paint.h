//
// Created by andy- on 2022-01-13.
//

#ifndef MCU1_GUI_PAINT_H
#define MCU1_GUI_PAINT_H

#include "fonts.h"
#include "EPD_2in66.h"

/*
 * Image attributes
 */
typedef struct {
    UBYTE *Image;
    UWORD Width;
    UWORD Height;
    UWORD WidthMemory;
    UWORD HeightMemory;
    UWORD Color;
    UWORD Rotate;
    UWORD Mirror;
    UWORD WidthByte;
    UWORD HeightByte;
    UWORD Scale;
} PAINT;

extern PAINT Paint;

/*
 * Display rotate
 */
#define ROATE_0             0
#define ROTATE_90           90
#define ROTATE_180          180
#define ROTATE_270          270

/*
 * Display Flip
 */
typedef enum {
    MIRROR_NONE = 0x00,
    MIRROR_HORIZONTAL = 0x01,
    MIRROR_VERTICAL = 0x02,
    MIRROR_ORIGIN = 0x03,
} MIRROR_IMAGE;

#define MIRROR_IMAGE_DFT MIRROR_NONE

/*
 * image color
 */
#define WHITE           0xFF
#define BLACK           0x00
#define RED             BLACK

#define IMAGE_BACKGROUND    WHITE
#define FONT_FOREGROUND     BLACK
#define FONT_BACKGROUND     WHITE

#define TRUE            1
#define FALSE           0

// 4 Gray level
#define GRAY1 0x03
#define GRAY2 0x02
#define GRAY3 0x01
#define GRAY4 0x00

/*
 * The size of the point
 */
typedef enum {
    DOT_PIXEL_1X1 = 1,
    DOT_PIXEL_2X2,
    DOT_PIXEL_3X3,
    DOT_PIXEL_4X4,
    DOT_PIXEL_5X5,
    DOT_PIXEL_6X6,
    DOT_PIXEL_7X7,
    DOT_PIXEL_8X8,
} DOT_PIXEL;
#define DOT_PIXEL_DFT DOT_PIXEL_1X1 // Default dot pixel

/*
 * Point size fill style
 */
typedef enum {
    DOT_FILL_AROUND = 1,
    DOT_FILL_RIGHTUP,
} DOT_STYLE;
#define DOT_STYLE_DFT DOT_FILL_AROUND  // Default dot pixel

/*
 * Line style, solid or dashed
 */
typedef enum {
    LINE_STYLE_SOLID = 0,
    LINE_STYLE_DOTTED
} LINE_STYLE;

/*
 * Whether the graphic is filled
 */
typedef enum {
    DRAW_FILL_EMPTY = 0,
    DRAW_FILL_FULL
} DRAW_FILL;

/*
 * Custom structure of a time attribute
 */
typedef struct {
    UWORD Year;
    UBYTE Month;
    UBYTE Day;
    UBYTE Hour;
    UBYTE Min;
    UBYTE Sec;
} PAINT_TIME;
extern PAINT_TIME sPaint_time;

// init and Clear
void Paint_NewImage(UBYTE *image, UWORD Width, UWORD Height, UWORD Rotate, UWORD Color);

void Paint_SelectImage(UBYTE *image);

void Paint_SetRotate(UWORD ROtate);

void Paint_SetMirroring(UBYTE mirror);

void Paint_SetPixel(UWORD Xpoint, UWORD Ypoint, UWORD Color);



#endif //MCU1_GUI_PAINT_H
