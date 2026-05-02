/**
 * tcs3472.hpp - highly optimized TCS3472 color sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef TCS3472_HPP
#define TCS3472_HPP

// include I2C bus control interface
#include <i2c.hpp>

struct TCS3472 {
    TCS3472(I2C *i2c, uint8_t address = 0x29);
    bool begin(bool interrupts = false);
    bool readColor(void* color, bool includeC = true);
private:
    I2C *_i2c;
    uint8_t _address;
};

#endif