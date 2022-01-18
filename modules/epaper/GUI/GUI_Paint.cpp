//
// Created by andy- on 2022-01-15.
//

#include "GUI_Paint.h"
#include "EPD_2in66.h"
#include "cstdint"
#include "cstdlib"
#include "cstring"
#include "cmath"

/*****************************************************************
 * function: Create Image
 * parameter:
 *      image   : Pointer ot the image cache
 *      Width   : The width of the picture
 *      Height  : The height of the picture
 *      Color   : Whether the picture is inverted
 *****************************************************************/
void Paint_NewImage(UBYTE *image, UWORD Width, UWORD Height, UWORD Rotate, UWORD Color){
    Paint.Image = nullptr;
    Paint.Image = image;

    Paint.WidthMemory = Width;
    Paint.HeightMemory = Height;
    Paint.Color = Color;
    Paint.Scale = 2;

    Paint.WidthByte = (Width % 8 == 0) ? (Width / 8) : (Width / 8 + 1);
    Paint.HeightByte = Height;

    Paint.Rotate = Rotate;
    Paint.Mirror = MIRROR_NONE;

    if (Rotate == ROTATE_0 || Rotate == ROTATE_180) {
        Paint.Width = Width;
        Paint.Height = Height;
    } else {
        Paint.Width = Height;
        Paint.Height = Width;
    }
}

/*****************************************************************
 * function: Select Image
 * parameter:
 *      image   : Pointer to the image cache
 *****************************************************************/
void Paint_SelectImage(UBYTE *image){
    Paint.Image = image;
}

/*****************************************************************
 * function: Set Image Rotate
 * parameter:
 *      Rotate  : 0, 90, 180, 270
 *****************************************************************/
void Paint_SetRotate(UWORD Rotate){
    if (Rotate == ROTATE_0 || Rotate == ROTATE_90 || Rotate == ROTATE_180 || Rotate == ROTATE_270) {
        // TODO: Debug
        Paint.Rotate = Rotate;
    } else {
        //TODO: Debug
    }
}

void Paint_SetScale(UBYTE scale){
    if (scale == 2) {
        Paint.Scale = scale;
        Paint.WidthByte = (Paint.WidthMemory % 8 == 0) ? (Paint.WidthMemory / 8) : (Paint.WidthMemory / 8 + 1);
    } else if (scale == 4) {
        Paint.Scale = scale;
        Paint.WidthByte = (Paint.WidthMemory % 4 == 0) ? (Paint.WidthMemory / 4) : (Paint.WidthMemory / 4 + 1);
    } else if (scale == 7) {
        Paint.Scale == scale;
        Paint.WidthMemory = (Paint.WidthMemory % 2 == 0) ? (Paint.WidthMemory / 2) : (Paint.WidthMemory / 2 + 1);
    } else {
        //TODO : Debug
    }
}

/******************************************************************************
function:	Set Image mirror
parameter:
    mirror   :Not mirror,Horizontal mirror,Vertical mirror,Origin mirror
******************************************************************************/
void Paint_SetMirroring(UBYTE mirror){
    if (mirror == MIRROR_NONE || mirror == MIRROR_HORIZONTAL ||
    mirror == MIRROR_VERTICAL || mirror == MIRROR_ORIGIN) {
        // TODO: Debug
        Paint.Mirror = mirror;
    } else {
        // TODO: Debug
    }
}

/******************************************************************************
function: Draw Pixels
parameter:
    Xpoint : At point X
    Ypoint : At point Y
    Color  : Painted colors
******************************************************************************/
void Paint_SetPixel(UWORD Xpoint, UWORD Ypoint, UWORD Color){
    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        // TODO: Debug
        return;
    }
    UWORD X, Y;
    switch(Paint.Rotate) {
        case 0:
            X = Xpoint;
            Y = Ypoint;
            break;
        case 90:
            X = Paint.WidthMemory - Ypoint - 1;
            Y = Xpoint;
        case 180:
            X = Paint.WidthMemory - Xpoint - 1;
            Y = Paint.HeightMemory - Ypoint - 1;
        case 270:
            X = Ypoint;
            Y = Paint.HeightMemory - Xpoint - 1;
            break;
        default:
            return;
    }

    switch (Paint.Mirror) {
        case MIRROR_NONE:
            break;
        case MIRROR_HORIZONTAL:
            X = Paint.WidthMemory - X - 1;
            break;
        case MIRROR_VERTICAL:
            Y = Paint.HeightMemory - Y - 1;
            break;
        case MIRROR_ORIGIN:
            X = Paint.WidthMemory - X - 1;
            Y = Paint.HeightMemory - Y - 1;
            break;
        default:
            return;
    }

    if(X > Paint.WidthMemory || Y > Paint.HeightMemory) {
        // TODO: Debug
        return;
    }
    if(Paint.Scale == 1) {
        UDOUBLE Addr = X / 8 + Y * Paint.WidthByte;
        UBYTE Rdata = Paint.Image[Addr];
        if(Color == BLACK) {
            Paint.Image[Addr] = Rdata & ~(0x80 >> (X % 8));
        } else {
            Paint.Image[Addr] = Rdata | (0x80 >> (X % 8));
        }
    } else if(Paint.Scale == 4) {
        UDOUBLE Addr = X / 4 + Y * Paint.WidthByte;
        Color = Color % 4; // Guaranteed color scale is 4 --- 0~3
        UBYTE Rdata = Paint.Image[Addr];
        Paint.Image[Addr] = Rdata | ((Color << 6) >> ((X % 4) * 2));
    } else if (Paint.Scale == 7) {
        UDOUBLE Addr = X / 2 + Y * Paint.WidthByte;
        UBYTE Rdata = Paint.Image[Addr];
        Rdata = Rdata & (~(0xF0 >> ((X % 2) * 4)));
        Paint.Image[Addr] = Rdata | ((Color << 4) >> ((X % 2) * 4));
    }
}

/******************************************************************************
function: Clear the color of the picture
parameter:
    Color : Painted colors
******************************************************************************/
void Paint_Clear(UWORD Color) {
    if(Paint.Scale == 2 || Paint.Scale == 4){
        for (UWORD Y = 0; Y < Paint.HeightByte; Y++) {
            for (UWORD X = 0; X < Paint.WidthByte; X++) {
                UDOUBLE Addr = X + Y * Paint.WidthByte;
                Paint.Image[Addr] = Color;
            }
        }
    } else if (Paint.Scale == 7) {
        for (UWORD Y = 0; Y < Paint.HeightByte; Y++) {
            for (UWORD X = 0; X < Paint.WidthByte; X++) {
                UDOUBLE Addr = X + Y * Paint.WidthByte;
                Paint.Image[Addr] = (Color << 4) | Color;
            }
        }
    }
}

/******************************************************************************
function: Clear the color of a window
parameter:
    Xstart : x starting point
    Ystart : Y starting point
    Xend   : x end point
    Yend   : y end point
    Color  : Painted colors
******************************************************************************/
void Paint_ClearWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD Color){
    UWORD X, Y;
    for (Y = Ystart; Y < Yend; Y++) {
        for (X = Xstart; X < Xend; X++) {
            Paint_SetPixel(X, Y, Color);
        }
    }
}

/******************************************************************************
function: Draw Point(Xpoint, Ypoint) Fill the color
parameter:
    Xpoint		: The Xpoint coordinate of the point
    Ypoint		: The Ypoint coordinate of the point
    Color		: Painted color
    Dot_Pixel	: point size
    Dot_Style	: point Style
******************************************************************************/
void Paint_DrawPoint(UWORD Xpoint, UWORD Ypoint, UWORD Color,
                     DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_Style){
    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        // TODO: Debug
        return;
    }
    int16_t XDir_Num, YDir_Num;
    if (Dot_Style == DOT_FILL_AROUND) {
        for (XDir_Num = 0; XDir_Num < 2 * Dot_Pixel - 1; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num < 2 * Dot_Pixel - 1; YDir_Num++) {
                if (Xpoint + XDir_Num - Dot_Pixel < 0 || Ypoint + YDir_Num - Dot_Pixel < 0) {
                    break;
                }
                Paint_SetPixel(Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel, Color);
            }
        }
    } else {
        for (XDir_Num = 0; XDir_Num < Dot_Pixel; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num < Dot_Pixel; YDir_Num++) {
                Paint_SetPixel(Xpoint + XDir_Num - 1, Ypoint + YDir_Num - 1, Color);
            }
        }
    }
}

/******************************************************************************
function: Draw a line of arbitrary slope
parameter:
    Xstart ：Starting Xpoint point coordinates
    Ystart ：Starting Xpoint point coordinates
    Xend   ：End point Xpoint coordinate
    Yend   ：End point Ypoint coordinate
    Color  ：The color of the line segment
    Line_width : Line width
    Line_Style: Solid and dotted lines
******************************************************************************/