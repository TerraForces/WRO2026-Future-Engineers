/**
 * st7735.hpp - highly optimized ST7735 controlled 160x128 pixels LCD control library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef ST7735_HPP
#define ST7735_HPP

#include <SPI.h>

// structure representing a monospace font used to render ASCII printable characters
struct font_t {
    uint8_t cw; // character width (in pixels)
    uint8_t ch; // character height (in pixels)
    uint8_t cd; // character distance - distance between the leftmost pixels of two consecutive characters on screen
    uint8_t lh; // line height - distance between the topmost pixels of two character lines
    const uint8_t* d; // pixel data - array of bytes which present one pixel line each (starting with char 32; saving in program memory recommended)
};

// object representing a ST7735 controlled 128x160 LCD display
struct ST7735 {

    /// @brief initializes display object, no communication with the display will be performed
    /// @param spi pointer to a SPI bus object used for communication with the display
    /// @param cs number of the chip select pin connected to the display
    /// @param dc number of the data/command pin connected to the display (sometimes labeled RS)
    /// @param rst number of the reset pin connected to the display (sometimes labeled RES)
    /// @param landscape whether to use landscape orientation on the display (uses portrait mode otherwise)
    ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape = true);

    /// @brief starts and configures the display controller
    /// @param r180 whether to rotate the display content by 180 degrees
    void begin(bool r180 = false);

    /// @brief draws a single character on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param f font to be used for the character (determines size and appearence)
    /// @param c character which should be drawn - only ASCII printable characters (character code 32-127) will be displayed
    /// @param fg foreground color used for the active pixels of the character (RGB565 color format)
    /// @param bg background color used to fill the remaining spaces (RGB565 color format)
    /// @param s scale / size factor - equals length of the side of one character pixel
    void drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint16_t bg, uint8_t s = 1);

    /// @brief draws a bitmap on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param w width of the bitmap
    /// @param h height of the bitmap
    /// @param c array of RGB565 color values (needs to contain w * h values)
    void drawImage(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t* c);

    /// @brief sets the color of a single pixel
    /// @param x x position of the pixel
    /// @param y y position of the pixel
    /// @param c color value which should be assigned (RGB565 color format)
    void drawPixel(uint8_t x, uint8_t y, uint16_t c);

    /// @brief draws a (solid filled) rectangle on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param w width of the rectangle
    /// @param h height of the rectangle
    /// @param c color used for all pixels (RGB565 color format)
    void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c);

    /// @brief draws a string of characters on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param f font to be used for the characters (determines size and appearence)
    /// @param t text / string of characters to be drawn - only ASCII printable characters (character code 32-127) will be displayed
    /// @param fg foreground color used for the active pixels of the characters (RGB565 color format)
    /// @param bg background color used to fill the remaining spaces (RGB565 color format)
    /// @param s scale / size factor - equals length of the side of one character pixel
    /// @param w whether to continue writing on a new line when the edge of the display is reached
    void drawText(uint8_t x, uint8_t y, font_t f, const char* t, uint16_t fg, uint16_t bg, uint8_t s = 1, bool w = true);

    /// @brief fills every pixel of the screen with the same color
    /// @param c color used for all pixels (RGB565 color format)
    void fillScreen(uint16_t c);
    
private:
    SPIClass* _spi;
    uint8_t _cs, _dc, _rst;
    uint8_t _w, _h;

    void displayInit(const uint8_t *addr); 
    void sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes);
    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

    // headless functions, no transaction start or end
    inline void writeCommand(uint8_t command);
};

#endif