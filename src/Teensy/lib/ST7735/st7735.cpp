/**
 * st7735.cpp - ST7735 controlled 160x128 pixels LCD control library
 * by TerraForces
 */

// include header file
#include "st7735.hpp"

// ST7735 startup/configuration process
static const uint8_t PROGMEM
startup[] = {               // 7735R init
    0x01, 0x80,             //  0: Software reset, 0 args, w/delay
    150,                    //     150 ms delay
    0x11, 0x80,             //  1: Out of sleep mode, 0 args, w/delay
    255,                    //     500 ms delay
    0xB1, 3,                //  2: Framerate ctrl - normal mode, 3 arg:
    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    0xB2, 3,                //  3: Framerate ctrl - idle mode, 3 args:
    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    0xB3, 6,                //  4: Framerate - partial mode, 6 args:
    0x01, 0x2C, 0x2D,       //     Dot inversion mode
    0x01, 0x2C, 0x2D,       //     Line inversion mode
    0xB4, 1,                //  5: Display inversion control, 1 arg:
    0x07,                   //     No inversion
    0xC0, 3,                //  6: Power control, 3 args, no delay:
    0xA2,
    0x02,                   //     -4.6V
    0x84,                   //     AUTO mode
    0xC1, 1,                //  7: Power control, 1 arg, no delay:
    0xC5,                   //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    0xC2, 2,                //  8: Power control, 2 args, no delay:
    0x0A,                   //     Opamp current small
    0x00,                   //     Boost frequency
    0xC3, 2,                //  9: Power control, 2 args, no delay:
    0x8A,                   //     BCLK/2,
    0x2A,                   //     opamp current small & medium low
    0xC4, 2,                // 10: Power control, 2 args, no delay:
    0x8A, 0xEE,
    0xC5, 1,                // 11: Power control, 1 arg, no delay:
    0x0E,
    0x20, 0,                // 12: Don't invert display, no args
    0x36, 1,                // 13: Mem access ctl (directions), 1 arg:
    0xC8,                   //     row/col addr, bottom-top refresh
    0x3A, 1,                // 14: set color mode, 1 arg, no delay:
    0x05,                   //     16-bit color
    0x2A, 4,                // 15: Column addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x7F,             //     XEND = 127
    0x2B, 4,                // 16: Row addr set, 4 args, no delay:
    0x00, 0x00,             //     XSTART = 0
    0x00, 0x9F,             //     XEND = 159s
    0xE0, 16,               // 17: Gamma Adjustments (pos. polarity), 16 args + delay:
    0x02, 0x1c, 0x07, 0x12, //     (Not entirely necessary, but provides
    0x37, 0x32, 0x29, 0x2d, //      accurate colors)
    0x29, 0x25, 0x2B, 0x39,
    0x00, 0x01, 0x03, 0x10,
    0xE1, 16,               // 18: Gamma Adjustments (neg. polarity), 16 args + delay:
    0x03, 0x1d, 0x07, 0x06, //     (Not entirely necessary, but provides
    0x2E, 0x2C, 0x29, 0x2D, //      accurate colors)
    0x2E, 0x2E, 0x37, 0x3F,
    0x00, 0x00, 0x02, 0x10,
    0x13, 0x80,             // 19: Normal display on, no args, w/delay
    10,                     //     10 ms delay
    0x29, 0x80,             // 20: Main screen turn on, no args w/delay
    100                     //     100 ms delay
};

// SPI settings for all transactions - 32 MHz
SPISettings settings = SPISettings(32000000, MSBFIRST, SPI_MODE0);

// set width and height based on orientation
ST7735::ST7735(SPIClass *spi, uint8_t cs, uint8_t dc, uint8_t rst, bool landscape) : _spi(spi), _cs(cs), _dc(dc), _rst(rst), _w(landscape ? 160 : 128), _h(landscape ? 128 : 160) {}

void ST7735::begin(bool r180) {

    // start SPI bus and reset display controller
    digitalWriteFast(_cs, HIGH);
    digitalWriteFast(_dc, HIGH);
    _spi->begin();
    digitalWriteFast(_rst, HIGH);
    delay(100);
    digitalWriteFast(_rst, LOW);
    delay(100);
    digitalWriteFast(_rst, HIGH);
    delay(200);

    // run the configuration process
    const uint8_t* addr = startup;
    uint8_t numCommands = 21;
    while (numCommands--) {
        uint8_t cmd = *(addr++);
        uint8_t numArgs = *(addr++);
        uint32_t ms = numArgs & 0x80;
        numArgs &= 0x7F;
        sendCommand(cmd, (void*)addr, numArgs);
        addr += numArgs;
        if (ms) {
            ms = *(addr++);
            delay((ms == 255) ? 500 : ms);
        }
    }

    // set display rotation
    uint8_t data = (_w == 160) ? (r180 ? 0xA0 : 0x60) : (r180 ? 0x00 : 0xC0);
    sendCommand(0x36, &data, 1);
}

void ST7735::drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint16_t bg, uint8_t s) {

    // only print supported (printable ASCII) characters
    if(c < 32) return;
    if(c > 126) return;
    c -= 32;

    // only fonts with max. 8 pixels of character width are supported
    if(f.cw > 8) return;

    // nothing to do when position is outside of the display
    if(x >= _w) return;
    if(y >= _h) return;

    // calculate scaled character size
    uint8_t w = f.cw * s;
    uint8_t h = f.ch * s;
    if(w == 0) return;
    if(h == 0) return;

    // only draw the visible part of the character
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;

    // get character bitmap
    uint8_t* b = (uint8_t*)f.d + (((f.cw + 7) / 8) * f.ch * c);

    // start SPI transaction
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);

    // send address window (write region)
    setAddrWindow(x, y, w, h);

    // save calculations when no scaling is needed
    if(s == 1) {

        // calculate last index that should be written
        uint8_t se = 8 - w;

        // count down rows while writing
        while(h--) {

            // write row of pixels
            uint8_t bb = *(b++);
            for(int8_t i = 7; i >= se; i--) {

                // print foreground color if pixel should be lit, background color otherwise
                transfer16((bb & (1 << i)) ? fg : bg);
            }
        }
    }
    else {

        // calculate last index that should be written (for w % s it's out of the display but ep is 0)
        uint8_t se = 7 - (w / s);

        // calculate when the character pixel row should be increased
        uint8_t hm = h % s;

        // calculate number of pixels drawn for the last character pixel
        uint8_t ep = w % s;

        // count down rows while writing
        while(h--) {

            // write row of pixels
            uint8_t bb = *b;
            for(int8_t i = 7; i >= se; i--) {
                bool lit = bb & (1 << i);
                for(uint8_t j = 0; j < ((i == se) ? ep : s); j++) transfer16(lit ? fg : bg);
            }

            // increment character row after writing s pixel rows
            if((h % s) == hm) b++;
        }
    }

    // end SPI transaction
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawChar(uint8_t x, uint8_t y, font_t f, char c, uint16_t fg, uint8_t s) {

    // only print supported (printable ASCII) characters
    if(c < 32) return;
    if(c > 126) return;
    c -= 32;

    // only fonts with max. 8 pixels of character width are supported
    if(f.cw > 8) return;

    // nothing to do when position is outside of the display
    if(x >= _w) return;
    if(y >= _h) return;

    // calculate scaled character size
    uint8_t w = f.cw * s;
    uint8_t h = f.ch * s;
    if(w == 0) return;
    if(h == 0) return;

    // only draw the visible part of the character
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;

    // get character bitmap
    uint8_t* b = (uint8_t*)f.d + (((f.cw + 7) / 8) * f.ch * c);

    // calculate last index that should be written
    uint8_t se = ((w % s) == 0) ? (8 - (w / s)) : (7 - (w / s));

    // count down rows while writing
    while(h) {

        // write row of pixels
        uint8_t bb = *(b++);
        for(int8_t i = 7; i >= se; i--) {

            // draw pixel only when lit (drawRect handles screen area fitting)
            if(bb & (1 << i)) drawRect(x, y, s, s, fg);
        }

        // decrease remaining rows by rows written
        if(h > s) h -= s;
        else h = 0;
    }
}

void ST7735::drawImage(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t* c) {

    // nothing to do when the area is 0 or outside of the display
    if(w == 0) return;
    if(h == 0) return;
    if(x >= _w) return;
    if(y >= _h) return;

    // calculate visible size on the display
    uint8_t dw = w;
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;
    dw -= w;

    // start SPI transaction
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);

    // send address window (write region)
    setAddrWindow(x, y, w, h);

    // transfer color data one row at a time
    while(h--) {

        // only draw until reaching the end of the line
        uint16_t* end = c + w;
        while(c < end) {

            // transfer color data
            transfer16(*(c++));
        }

        // increment color pointer by the number of pixels that have been cut off
        c += dw;
    }

    // end SPI transaction
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawPixel(uint8_t x, uint8_t y, uint16_t c) {

    // nothing to do when the pixel is outside of the display
    if(x >= _w) return;
    if(y >= _h) return;

    // start SPI transaction
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);

    // send address window (write region)
    setAddrWindow(x, y, 1, 1);

    // transfer color data
    transfer16(c);

    // end SPI transaction
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c) {

    // nothing to do when the area is 0 or outside of the display
    if(w == 0) return;
    if(h == 0) return;
    if(x >= _w) return;
    if(y >= _h) return;

    // calculate visible size on the display
    if((x + w) > _w) w = _w - x;
    if((y + h) > _h) h = _h - y;

    // calculate area
    uint32_t i = w * h;

    // start SPI transaction
    _spi->beginTransaction(settings);
    digitalWriteFast(_cs, LOW);

    // send address window (write region)
    setAddrWindow(x, y, w, h);

    // transfer color data for every pixel
    while(i--) transfer16(c);

    // end SPI transaction
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}

void ST7735::drawText(uint8_t x, uint8_t y, font_t f, const char* t, uint16_t fg, uint16_t bg, uint8_t s, bool w) {

    // calculate scaled character spacings
    uint8_t dx = f.cd * s;
    uint8_t dy = f.lh * s;

    // current x position ("cursor")
    uint8_t p = x;

    // calculate the maximum position value where drawing is possible
    uint8_t mp = w ? (_w - (f.cw * s)) : _w;

    // write until the character t is 0 (string termination)
    while(*t) {

        // move cursor to the start of the next line when t is a newline character (10) or end drawing when it would be outside of the screen
        if(*t == '\n') {
            y += dy;
            if(y >= _h) return;
            p = x;
        }
        else {

            // draw char and increment position if it still fits on the line
            if(p < mp) {
                drawChar(p, y, f, *t, fg, bg, s);
                p += dx;
            }

            // if text wrapping is enabled, continue writing on the next line if available
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

    // draw a rectangle in color c over the entire screen
    drawRect(0, 0, _w, _h, c);
}

void ST7735::sendCommand(uint8_t command, void *data, uint8_t dataSize, bool completeTransaction) {

    // start SPI transaction if requested
    if(completeTransaction) {
        _spi->beginTransaction(settings);
        digitalWriteFast(_cs, LOW);
    }

    // hold DC low while sending the command byte
    digitalWriteFast(_dc, LOW);
    _spi->transfer(command);
    digitalWriteFast(_dc, HIGH);

    // transfer payload data if available
    //if(dataSize) _spi->transfer(data, dataSize);
    for(uint8_t i = 0; i < dataSize; i++) _spi->transfer(((uint8_t*)data)[i]);

    // end SPI transaction if requested
    if(completeTransaction) {
        digitalWriteFast(_cs, HIGH);
        _spi->endTransaction();
    }
}

void ST7735::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {

    // set x range (start, end)
    sendCommand(0x2A, 0, 0, false);
    transfer16(x);
    transfer16(x + w - 1);

    // set y range (start, end)
    sendCommand(0x2B, 0, 0, false);
    transfer16(y);
    transfer16(y + h - 1);

    // apply address window
    sendCommand(0x2C, 0, 0, false);
}

void ST7735::transfer16(uint16_t d) {
    _spi->transfer(d >> 8);
    _spi->transfer(d & 0xFF);
}