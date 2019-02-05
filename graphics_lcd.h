// Graphics LCD Driver
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// ST7565R Graphics LCD Display Interface:
//   MOSI (SSI2Tx) on PB7
//   MISO (SSI2Rx) is not used by the LCD display but the pin is used for GPIO for A0
//   SCLK (SSI2Clk) on PB4
//   A0 connected to PB6
//   ~CS connected to PB1

#ifndef GRAPHICS_LCD_H_
#define GRAPHICS_LCD_H_

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2

// Pins
#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void clearGraphicsLcd();
void initGraphicsLcd();
void drawGraphicsLcdPixel(uint8_t x, uint8_t y, uint8_t op);
void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, uint8_t op);
void setGraphicsLcdTextPosition(uint8_t x, uint8_t page);
void putcGraphicsLcd(char c);
void putsGraphicsLcd(char str[]);

#endif

