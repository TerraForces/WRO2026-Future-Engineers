/**
 * tcs3472.cpp - highly optimized TCS3472 color sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "tcs3472.hpp"

TCS3472::TCS3472(I2C *i2c, uint8_t address) : _i2c(i2c), _address(address) {}

bool TCS3472::begin(bool interrupts) {
    uint8_t data = interrupts ? 0x13 : 0x03;
    return _i2c->writeRegs(_address, 0x00, &data, 1);
}

bool TCS3472::readColor(void* color, bool includeC) {
    return _i2c->readRegs(_address, includeC ? 0xB4: 0xB6, color, includeC ? 8 : 6, true);
}