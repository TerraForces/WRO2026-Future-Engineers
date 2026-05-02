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

struct font_t {
    uint8_t cw, ch, cd, lh;
    const uint8_t* d;
};

struct ST7735 {
    ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape = true, bool blockBus = false);
    void begin(bool r180 = false);
    void drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint16_t bg, uint8_t s = 1);
    void drawImage(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t* c);
    void drawPixel(uint8_t x, uint8_t y, uint16_t c);
    void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c);
    void drawText(uint8_t x, uint8_t y, font_t f, const char* t, uint16_t fg, uint16_t bg, uint8_t s = 1, bool w = true);
    void fillScreen(uint16_t c);
private:
    SPIClass* _spi;
    uint8_t _cs, _dc, _rst;
    uint8_t _w, _h;
    bool _blockBus;

    void displayInit(const uint8_t *addr); 
    void sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes);
    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

    // headless functions, no transaction start or end
    inline void writeCommand(uint8_t command);
};

#endif