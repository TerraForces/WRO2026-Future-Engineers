/**
 * st7735.hpp - highly optimized ST7735 LCD control library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "st7735.hpp"

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1 0xDA
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD

#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

static const uint8_t PROGMEM
Rcmd1[] = {                 // 7735R init, part 1 (red or green tab)
    15,                     // 15 commands in list:
    0x01, 0x80,             //  1: Software reset, 0 args, w/delay
    150,                    //     150 ms delay
    0x11, 0x80,             //  2: Out of sleep mode, 0 args, w/delay
    255,                    //     500 ms delay
    0xB1, 3,                //  3: Framerate ctrl - normal mode, 3 arg:
    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    0xB2, 3,                //  4: Framerate ctrl - idle mode, 3 args:
    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    0xB3, 6,                //  5: Framerate - partial mode, 6 args:
    0x01, 0x2C, 0x2D,       //     Dot inversion mode
    0x01, 0x2C, 0x2D,       //     Line inversion mode
    0xB4, 1,                //  6: Display inversion ctrl, 1 arg:
    0x07,                   //     No inversion
    0xC0, 3,                //  7: Power control, 3 args, no delay:
    0xA2,
    0x02,                   //     -4.6V
    0x84,                   //     AUTO mode
    0xC1, 1,                //  8: Power control, 1 arg, no delay:
    0xC5,                   //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    0xC2, 2,                //  9: Power control, 2 args, no delay:
    0x0A,                   //     Opamp current small
    0x00,                   //     Boost frequency
    0xC3, 2,                // 10: Power control, 2 args, no delay:
    0x8A,                   //     BCLK/2,
    0x2A,                   //     opamp current small & medium low
    0xC4, 2,                // 11: Power control, 2 args, no delay:
    0x8A, 0xEE,
    0xC5, 1,                // 12: Power control, 1 arg, no delay:
    0x0E,
    0x20, 0,                // 13: Don't invert display, no args
    0x36, 1,                // 14: Mem access ctl (directions), 1 arg:
    0xC8,                   //     row/col addr, bottom-top refresh
    0x3A, 1,                // 15: set color mode, 1 arg, no delay:
    0x05                    //     16-bit color
},                       
Rcmd2red[] = {              // 7735R init, part 2 (red tab only)
    2,                      //  2 commands in list:
    0x2A, 4,                //  1: Column addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x7F,             //     XEND = 127
    0x2B, 4,                //  2: Row addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x9F              //     XEND = 159s
},                 
Rcmd3[] = {                 // 7735R init, part 3 (red or green tab)
    4,                      //  4 commands in list:
    0xE0, 16,               //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
    0x02, 0x1c, 0x07, 0x12, //     (Not entirely necessary, but provides
    0x37, 0x32, 0x29, 0x2d, //      accurate colors)
    0x29, 0x25, 0x2B, 0x39,
    0x00, 0x01, 0x03, 0x10,
    0xE1, 16,               //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
    0x03, 0x1d, 0x07, 0x06, //     (Not entirely necessary, but provides
    0x2E, 0x2C, 0x29, 0x2D, //      accurate colors)
    0x2E, 0x2E, 0x37, 0x3F,
    0x00, 0x00, 0x02, 0x10,
    0x13, 0x80,             //  3: Normal display on, no args, w/delay
    10,                     //     10 ms delay
    0x29, 0x80,             //  4: Main screen turn on, no args w/delay
    100                     //     100 ms delay
};
SPISettings settings = SPISettings(32000000, MSBFIRST, SPI_MODE0);

ST7735::ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape, bool blockBus) : _spi(spi), _cs(cs), _dc(dc), _rst(rst), _w(landscape ? 160 : 128), _h(landscape ? 128 : 160), _blockBus(blockBus) {}

void ST7735::begin(bool r180) {
    digitalWriteFast(_cs, HIGH);
    digitalWriteFast(_dc, HIGH);
    _spi->begin();
    digitalWriteFast(_rst, HIGH);
    delay(100);
    digitalWriteFast(_rst, LOW);
    delay(100);
    digitalWriteFast(_rst, HIGH);
    delay(200);
    displayInit(Rcmd1);
    displayInit(Rcmd2red);
    displayInit(Rcmd3);
    uint8_t data = (_w == 160) ? (r180 ? 0xA0 : 0x60) : (r180 ? 0x00 : 0xC0);
    sendCommand(0x36, &data, 1);
}

#include <limits.h>

void ST7735::drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint16_t bg, uint8_t s) {
#ifndef __CHAR_UNSIGNED__
    c += 128;
#endif
    if(c < 32) return;
    if(c > 126) return;
    c -= 32;
    if(f.cw > 8) return;
    if(x >= _w) return;
    if(y >= _h) return;
    uint8_t w = f.cw * s;
    uint8_t h = f.ch * s;
    if(w == 0) return;
    if(h == 0) return;
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;
    uint8_t fg0 = fg >> 8;
    uint8_t fg1 = fg;
    uint8_t bg0 = bg >> 8;
    uint8_t bg1 = bg;
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);
    setAddrWindow(x, y, w, h);
    uint8_t* b = (uint8_t*)f.d + (((f.cw + 7) / 8) * f.ch * c);
    if(s == 1) {
        uint8_t se = 8 - w;
        while(h--) {
            uint8_t bb = *b;
            for(int8_t i = 7; i >= se; i--) {
                if(bb & (1 << i)) {
                    _spi->transfer(fg0);
                    _spi->transfer(fg1);
                }
                else {
                    _spi->transfer(bg0);
                    _spi->transfer(bg1);
                }
            }
            b++;
        }
    }
    else {
        uint8_t hm = h % s;
        uint8_t se = 7 - (w / s);
        uint8_t ep = w % s;
        while(h--) {
            uint8_t bb = *b;
            for(int8_t i = 7; i >= se; i--) {
                if(bb & (1 << i)) {
                    for(uint8_t j = 0; j < ((i == se) ? ep : s); j++) {
                        _spi->transfer(fg0);
                        _spi->transfer(fg1);
                    }
                }
                else {
                    for(uint8_t j = 0; j < ((i == se) ? ep : s); j++) {
                        _spi->transfer(bg0);
                        _spi->transfer(bg1);
                    }
                }
            }
            if(h % s == hm) b++;
        }
    }
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawImage(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t* c) {
    if(w == 0) return;
    if(h == 0) return;
    if(x >= _w) return;
    if(y >= _h) return;
    uint8_t dw = w;
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;
    dw -= w;
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);
    setAddrWindow(x, y, w, h);
    while(h--) {
        uint16_t* end = c + w;
        while(c < end) {
            _spi->transfer(*c >> 8);
            _spi->transfer(*c);
            c++;
        }
        c += dw;
    }
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawPixel(uint8_t x, uint8_t y, uint16_t c) {
    if(x >= _w) return;
    if(y >= _h) return;
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);
    setAddrWindow(x, y, 1, 1);
    _spi->transfer(c >> 8);
    _spi->transfer(c);
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c) {
    if(w == 0) return;
    if(h == 0) return;
    if(x >= _w) return;
    if(y >= _h) return;
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;
    uint8_t c0 = c >> 8;
    uint8_t c1 = c;
    uint16_t i = w * h;
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);
    setAddrWindow(x, y, w, h);
    while(i--) {
        _spi->transfer(c0);
        _spi->transfer(c1);
    }
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawText(uint8_t x, uint8_t y, font_t f, const char* t, uint16_t fg, uint16_t bg, uint8_t s, bool w) {
    uint8_t dx = f.cd * s;
    uint8_t dy = f.lh * s;
    uint8_t p = x;
    uint8_t mw = w ? (_w - (f.cw * s)) : _w;
    while(*t) {
        if(*t == '\n') {
            y += dy;
            if(y >= _h) return;
            p = x;
        }
        else {
            if(p < mw) {
                drawChar(p, y, f, *t, fg, bg, s);
                p += dx;
            }
            else if(w) {
                y += dy;
                if(y >= _h) return;
                p = x;
                drawChar(p, y, f, *t, fg, bg, s);
                p += dx;
            }
        }
        t++;
    }
}

void ST7735::fillScreen(uint16_t c) {
    drawRect(0, 0, _w, _h, c);
}

void ST7735::displayInit(const uint8_t *addr) {
    uint8_t numCommands, cmd, numArgs;
    uint16_t ms;
    numCommands = *(addr++); // Number of commands to follow
    while (numCommands--) {              // For each command...
        cmd = *(addr++);       // Read command
        numArgs = *(addr++);   // Number of args to follow
        ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
        sendCommand(cmd, addr, numArgs);
        addr += numArgs;
        if (ms) {
            ms = *(addr++); // Read post-command delay time (ms)
            if (ms == 255) ms = 500; // If 255, delay for 500 ms
            delay(ms);
        }
    }
}

void ST7735::sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);
    digitalWriteFast(_dc, LOW);
    _spi->transfer(commandByte);
    digitalWriteFast(_dc, HIGH);
    for (int i = 0; i < numDataBytes; i++) _spi->transfer(*(dataBytes++));
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    writeCommand(ST77XX_CASET);
    _spi->transfer(x >> 8);
    _spi->transfer(x);
    uint16_t a = x + w - 1;
    _spi->transfer(a >> 8);
    _spi->transfer(a);
    writeCommand(ST77XX_RASET);
    _spi->transfer(y >> 8);
    _spi->transfer(y);
    uint16_t b = y + h - 1;
    _spi->transfer(b >> 8);
    _spi->transfer(b);
    writeCommand(ST77XX_RAMWR);
}

inline void ST7735::writeCommand(uint8_t command) {
    digitalWriteFast(_dc, LOW);
    _spi->transfer(command);
    digitalWriteFast(_dc, HIGH);
}