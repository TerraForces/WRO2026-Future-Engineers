/**
 * st7735.hpp - highly optimized ST7735 LCD control library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "st7735.hpp"

// ST7735 startup/configuration process
static const uint8_t PROGMEM
startup[] = {               // 7735R init
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
    0x05,                   //     16-bit color
    0x2A, 4,                // 16: Column addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x7F,             //     XEND = 127
    0x2B, 4,                // 17: Row addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x9F,             //     XEND = 159s
    0xE0, 16,               // 18: Gamma Adjustments (pos. polarity), 16 args + delay:
    0x02, 0x1c, 0x07, 0x12, //     (Not entirely necessary, but provides
    0x37, 0x32, 0x29, 0x2d, //      accurate colors)
    0x29, 0x25, 0x2B, 0x39,
    0x00, 0x01, 0x03, 0x10,
    0xE1, 16,               // 19: Gamma Adjustments (neg. polarity), 16 args + delay:
    0x03, 0x1d, 0x07, 0x06, //     (Not entirely necessary, but provides
    0x2E, 0x2C, 0x29, 0x2D, //      accurate colors)
    0x2E, 0x2E, 0x37, 0x3F,
    0x00, 0x00, 0x02, 0x10,
    0x13, 0x80,             // 20: Normal display on, no args, w/delay
    10,                     //     10 ms delay
    0x29, 0x80,             // 21: Main screen turn on, no args w/delay
    100                     //     100 ms delay
};
SPISettings settings = SPISettings(32000000, MSBFIRST, SPI_MODE0);

ST7735::ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape) : _spi(spi), _cs(cs), _dc(dc), _rst(rst), _w(landscape ? 160 : 128), _h(landscape ? 128 : 160) {}

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
    const uint8_t* addr = startup;
    uint8_t numCommands = 21;
    while (numCommands--) {
        uint8_t cmd = *(addr++);
        uint8_t numArgs = *(addr++);
        uint16_t ms = numArgs & 0x80;
        numArgs &= ~0x80;
        sendCommand(cmd, (void*)addr, numArgs);
        addr += numArgs;
        if (ms) {
            ms = *(addr++);
            if (ms == 255) ms = 500;
            delay(ms);
        }
    }

    // set display rotation
    uint8_t data = (_w == 160) ? (r180 ? 0xA0 : 0x60) : (r180 ? 0x00 : 0xC0);
    sendCommand(0x36, &data, 1);
}

void ST7735::drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint16_t bg, uint8_t s) {
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

void ST7735::sendCommand(uint8_t command, void *data, uint8_t dataSize, bool completeTransaction) {
    if(completeTransaction) {
        _spi->beginTransaction(settings);
        digitalWriteFast(_cs, LOW);
    }
    digitalWriteFast(_dc, LOW);
    _spi->transfer(command);
    digitalWriteFast(_dc, HIGH);
    for(uint8_t i = 0; i < dataSize; i++) _spi->transfer(((uint8_t*)data)[i]);
    if(completeTransaction) {
        digitalWriteFast(_cs, HIGH);
        _spi->endTransaction();
    }
}

void ST7735::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    sendCommand(0x2A, 0, 0, false);
    _spi->transfer(x >> 8);
    _spi->transfer(x);
    uint16_t a = x + w - 1;
    _spi->transfer(a >> 8);
    _spi->transfer(a);
    sendCommand(0x2B, 0, 0, false);
    _spi->transfer(y >> 8);
    _spi->transfer(y);
    uint16_t b = y + h - 1;
    _spi->transfer(b >> 8);
    _spi->transfer(b);
    sendCommand(0x2C, 0, 0, false);
}