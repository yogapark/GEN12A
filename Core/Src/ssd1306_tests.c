#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"

//------------------------------------------------------------------------------
// Table generated by LCD Assistant
// http://en.radzio.dxp.pl/bitmap_converter/
//------------------------------------------------------------------------------
const unsigned char garfield_128x64 [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xCF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFB, 0xE3, 0xFF, 0xF3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0, 0xF3, 0xFF, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x1F, 0xD9, 0xFC, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x0F, 0x00, 0x1E, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0E, 0x1F, 0xF8, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0x87, 0xF0, 0x07, 0x00, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x18, 0x3B, 0x80, 0x18, 0x00, 0x30, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x60, 0x00, 0x18, 0xC3, 0x01, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x01, 0x80, 0x00, 0x0F, 0xC3, 0x8F, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x07, 0xF0, 0x03, 0x00, 0x00, 0x0C, 0x07, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x0C, 0x00, 0x00, 0x0C, 0x07, 0xFF, 0xF8, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x38, 0xC0, 0x18, 0x00, 0x00, 0x08, 0x0F, 0xFF, 0xF0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x71, 0x80, 0x30, 0x00, 0x00, 0x08, 0x0F, 0xFC, 0xFC, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xF7, 0x00, 0x60, 0x00, 0x00, 0x18, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x60, 0x00, 0x00, 0x18, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0xC0, 0x00, 0x00, 0x30, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x83, 0xE0, 0xC0, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0x03, 0x1F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0E, 0x01, 0x00, 0xE7, 0xC0, 0x03, 0x80, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0F, 0x01, 0x80, 0xE0, 0x00, 0x06, 0x00, 0x00, 0xFF, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0F, 0x83, 0x7F, 0xF0, 0x00, 0x38, 0x00, 0x26, 0xFF, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0F, 0xDC, 0x18, 0x1F, 0x87, 0xE0, 0x00, 0x26, 0xEF, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0F, 0xB8, 0x0E, 0x18, 0x00, 0x30, 0x00, 0x04, 0xCF, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0F, 0x30, 0x0F, 0xC0, 0x00, 0x18, 0x00, 0x00, 0x8F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0x30, 0x38, 0x70, 0x00, 0x18, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0x18, 0x00, 0x1F, 0x00, 0x18, 0x3B, 0x80, 0x0E, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x8E, 0x00, 0x03, 0x80, 0x70, 0x1F, 0xF8, 0x1C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x03, 0xE0, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x7F, 0x86, 0x00, 0x3F, 0x1F, 0x07, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0F, 0xC3, 0x00, 0x00, 0x3F, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x3F, 0xC7, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0xE0, 0x07, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x07, 0xC0, 0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0xA4, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x16, 0x01, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x1E, 0x01, 0xC0, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x0C, 0x01, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x04, 0x01, 0xF3, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0C, 0x01, 0xEF, 0xF7, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x18, 0x00, 0x07, 0xF0, 0x60, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x18, 0x00, 0x07, 0x80, 0x70, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x30, 0x00, 0x06, 0x04, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x60, 0x00, 0x0C, 0x1C, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01, 0xFD, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xFD, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0xC1, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x07, 0x00, 0x00, 0x03, 0x81, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x39, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x01, 0xF0, 0x01, 0xE3, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x1F, 0x03, 0x8D, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x03, 0xFC, 0x3D, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x00, 0x03, 0xC0, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xC0, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xF1, 0xC0, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xE0, 0xC0, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xE0, 0xC0, 0x3F, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char github_logo_64x64[] = {
0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x80, 0x00, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00,
0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x03, 0xFE, 0x3F, 0xFF, 0xFF, 0xFC, 0x7F, 0xC0,
0x07, 0xFE, 0x07, 0xFF, 0xFF, 0xE0, 0x7F, 0xE0, 0x07, 0xFE, 0x01, 0xF8, 0x1F, 0xC0, 0x3F, 0xE0,
0x0F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x0F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0,
0x1F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0x1F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8,
0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFC,
0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE,
0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE,
0x7F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF,
0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF,
0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE,
0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE,
0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFC,
0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFC, 0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x01, 0xFF, 0xFC,
0x1F, 0x9F, 0xE0, 0x00, 0x00, 0x07, 0xFF, 0xF8, 0x1F, 0x87, 0xFC, 0x00, 0x00, 0x3F, 0xFF, 0xF8,
0x0F, 0xC3, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xF0, 0x0F, 0xE1, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0xF0,
0x07, 0xF1, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0xE0, 0x07, 0xF0, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xE0,
0x03, 0xF8, 0x3F, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x01, 0xF8, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x80,
0x00, 0xFC, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00,
0x00, 0x7F, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0xFF, 0xFC, 0x00,
0x00, 0x0F, 0xFF, 0x00, 0x00, 0xFF, 0xF0, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0xFF, 0xE0, 0x00,
0x00, 0x03, 0xFF, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00,
0x00, 0x00, 0x3F, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xF0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void ssd1306_TestBorder() {
    ssd1306_Fill(Black);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    uint8_t x = 0;
    uint8_t y = 0;
    do {
        ssd1306_DrawPixel(x, y, Black);

        if((y == 0) && (x < 127))
            x++;
        else if((x == 127) && (y < (SSD1306_HEIGHT-1)))
            y++;
        else if((y == (SSD1306_HEIGHT-1)) && (x > 0)) 
            x--;
        else
            y--;

        ssd1306_DrawPixel(x, y, White);
        ssd1306_UpdateScreen();
    
        HAL_Delay(5);
        end = HAL_GetTick();
    } while((end - start) < 8000);
   
    HAL_Delay(1000);
}

void ssd1306_TestFonts() {
    uint8_t y = 0;
    ssd1306_Fill(Black);

    #ifdef SSD1306_INCLUDE_FONT_16x26
    ssd1306_SetCursor(2, y);
    ssd1306_WriteString("Font 16x26", Font_16x26, White);
    y += 26;
    #endif

    #ifdef SSD1306_INCLUDE_FONT_11x18
    ssd1306_SetCursor(2, y);
    ssd1306_WriteString("Font 11x18", Font_11x18, White);
    y += 18;
    #endif

    #ifdef SSD1306_INCLUDE_FONT_7x10
    ssd1306_SetCursor(2, y);
    ssd1306_WriteString("Font 7x10", Font_7x10, White);
    y += 10;
    #endif

    #ifdef SSD1306_INCLUDE_FONT_6x8
    ssd1306_SetCursor(2, y);
    ssd1306_WriteString("Font 6x8", Font_6x8, White);
    #endif

    ssd1306_UpdateScreen();
}

void ssd1306_TestFPS() {
    ssd1306_Fill(White);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    int fps = 0;
    char message[] = "ABCDEFGHIJK";
   
    ssd1306_SetCursor(2,0);
    ssd1306_WriteString("Testing...", Font_11x18, Black);
    ssd1306_SetCursor(2, 18*2);
    ssd1306_WriteString("0123456789A", Font_11x18, Black);
   
    do {
        ssd1306_SetCursor(2, 18);
        ssd1306_WriteString(message, Font_11x18, Black);
        ssd1306_UpdateScreen();
       
        char ch = message[0];
        memmove(message, message+1, sizeof(message)-2);
        message[sizeof(message)-2] = ch;

        fps++;
        end = HAL_GetTick();
    } while((end - start) < 5000);
   
    HAL_Delay(5000);

    char buff[64];
    fps = (float)fps / ((end - start) / 1000.0);
    snprintf(buff, sizeof(buff), "~%d FPS", fps);
   
    ssd1306_Fill(White);
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString(buff, Font_11x18, Black);
    ssd1306_UpdateScreen();
}

void ssd1306_TestLine() {

  ssd1306_Line(1,1,SSD1306_WIDTH - 1,SSD1306_HEIGHT - 1,White);
  ssd1306_Line(SSD1306_WIDTH - 1,1,1,SSD1306_HEIGHT - 1,White);
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestRectangle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawRectangle(1 + (5*delta),1 + (5*delta) ,SSD1306_WIDTH-1 - (5*delta),SSD1306_HEIGHT-1 - (5*delta),White);
  }
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestCircle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawCircle(20* delta+30, 15, 10, White);
  }
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestArc() {
  ssd1306_DrawArc(60, 30, 30, 300, 360, White);
  ssd1306_DrawArc(60, 30, 30, 0, 180, White);
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestPolyline() {
  SSD1306_VERTEX loc_vertex[] =
  {
      {35,40},
      {40,20},
      {45,28},
      {50,10},
      {45,16},
      {50,10},
      {53,16}
  };

  ssd1306_Polyline(loc_vertex,sizeof(loc_vertex)/sizeof(loc_vertex[0]),White);
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestDrawBitmap()
{
    ssd1306_Fill(White);
    ssd1306_DrawBitmap(0,0,garfield_128x64,128,64,Black);
    ssd1306_UpdateScreen();
    HAL_Delay(3000);
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(32,0,github_logo_64x64,64,64,White);
    ssd1306_UpdateScreen();
    HAL_Delay(3000);
    ssd1306_Fill(White);
    ssd1306_DrawBitmap(32,0,github_logo_64x64,64,64,Black);
    ssd1306_UpdateScreen();
}

void ssd1306_TestAll() {
    ssd1306_Init();
    ssd1306_TestFPS();
    HAL_Delay(3000);
//    ssd1306_TestBorder();
    ssd1306_TestFonts();
    HAL_Delay(3000);
    ssd1306_Fill(Black);
    ssd1306_TestRectangle();
    ssd1306_TestLine();
    HAL_Delay(3000);
//    ssd1306_Fill(Black);
//    ssd1306_TestPolyline();
//    HAL_Delay(3000);
//    ssd1306_Fill(Black);
//    ssd1306_TestArc();
//    HAL_Delay(3000);
//    ssd1306_Fill(Black);
//    ssd1306_TestCircle();
//    HAL_Delay(3000);
    ssd1306_TestDrawBitmap();
    HAL_Delay(3000);
}

