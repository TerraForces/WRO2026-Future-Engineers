/**
 * tcs3472.cpp - highly optimized TCS3472 color sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "tcs3472.hpp"

TCS3472::TCS3472(I2C *i2c, uint8_t address) : _i2c(i2c), _address(address) {}

bool TCS3472::begin() {

    // set integration time to 2.4 ms (register 0x01)
    uint8_t data = 0xFF;
    if(!_i2c->writeRegs(_address, 0x81, &data, 1)) return false;

    // set gain to 16x (register 0x0F)
    data = 0x02;
    if(!_i2c->writeRegs(_address, 0x8F, &data, 1)) return false;

    // power on and enable ADCs (register 0x00)
    data = 0x03;
    return _i2c->writeRegs(_address, 0x80, &data, 1);
}

bool TCS3472::readColor(void* color, bool includeC) {

    // read color data continously from the data registers (starting at 0x14 with clear or 0x16 with red)
    return _i2c->readRegs(_address, includeC ? 0xB4: 0xB6, color, includeC ? 8 : 6, true);
}