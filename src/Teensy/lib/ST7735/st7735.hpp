/**
 * st7735.hpp - ST7735 controlled 160x128 pixels LCD control library
 * by TerraForces
 */

// prevent including multiple times
#ifndef ST7735_HPP
#define ST7735_HPP

#include <SPI.h>

// macro to convert 8-bit/channel (RGB888) color values to RGB565
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3))

// structure representing a monospace font used to render ASCII printable characters
struct font_t {
    uint8_t cw; // character width (in pixels)
    uint8_t ch; // character height (in pixels)
    uint8_t cd; // character distance - horizontal distance between the leftmost pixels of two consecutive characters on screen
    uint8_t lh; // line height - vertical distance between the topmost pixels of two character lines
    const uint8_t* d; // pixel data - array of bytes which each represent one pixel row (only chars 32 - 126)
};

// object representing a ST7735 controlled 128x160 LCD display
struct ST7735 {

    /// @brief initializes display object, no communication with the display will be performed
    /// @param spi pointer to a SPI bus object for communication with the display
    /// @param cs number of the chip select pin connected to the display
    /// @param dc number of the data/command pin connected to the display (sometimes labeled RS)
    /// @param rst number of the reset pin connected to the display (sometimes labeled RES)
    /// @param landscape whether to use landscape orientation on the display (uses portrait mode if false)
    ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape = true);

    /// @brief starts and configures the display controller
    /// @param r180 whether to rotate the display content by 180 degrees
    void begin(bool r180 = false);

    /// @brief draws a single character on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param f font to be used for the character (determines size and appearence)
    /// @param c character which should be drawn - only ASCII printable characters (character code 32-126) will be displayed
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

    /// @brief draws a rectangle (solid fill) on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param w width of the rectangle
    /// @param h height of the rectangle
    /// @param c color used for all pixels (RGB565 color format)
    void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c);

    /// @brief draws multiple characters (zero terminated) on the display
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param f font to be used for the characters (determines size and appearence)
    /// @param t characters to be drawn - only ASCII printable characters (character code 32-127) will be displayed, writing stops when reaching character value 0
    /// @param fg foreground color used for the active pixels of the characters (RGB565 color format)
    /// @param bg background color used to fill the remaining spaces (RGB565 color format)
    /// @param s scale / size factor - equals length of the side of one character pixel
    /// @param w whether to continue writing on a new line when the edge of the display is reached
    void drawText(uint8_t x, uint8_t y, font_t f, const char* t, uint16_t fg, uint16_t bg, uint8_t s = 1, bool w = true);

    /// @brief fills every pixel of the screen with the same color
    /// @param c color used for all pixels (RGB565 color format)
    void fillScreen(uint16_t c);
    
private:
    SPIClass* _spi;         // pointer to the SPI bus object
    uint8_t _cs, _dc, _rst; // numbers of MCU pins connected to the display
    uint8_t _w, _h;         // display dimensions

    /// @brief send a command byte followed by data bytes
    /// @param command byte which should be sent in command mode
    /// @param data bytes which should be sent in data mode
    /// @param dataSize size of the data to be sent in bytes
    /// @param completeTransaction whether a new transaction should be started before and finished after the transfer
    void sendCommand(uint8_t command, void *data, uint8_t dataSize, bool completeTransaction = true);

    /// @brief set address window for the next color data (SPI transaction has to be running)
    /// @param x x position of the leftmost pixels
    /// @param y y position of the topmost pixels
    /// @param w width of the address window
    /// @param h height of the address window
    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
};

#endif