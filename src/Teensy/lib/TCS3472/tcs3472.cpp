/**
 * tcs3472.cpp - TCS3472 color sensor library
 * by TerraForces
 */

// include header file
#include "tcs3472.hpp"

TCS3472::TCS3472(I2C& i2c, uint8_t address) : _i2c(i2c), _address(address) {}

bool TCS3472::begin() {

    // note: all command bytes must have bit 7 set and optionally bit 5 to auto-increment the registers while reading / writing

    // set integration time to 2.4 ms (register 0x01)
    uint8_t data = 0xFF; // data = 256 - (integration time / 2.4 ms)
    if(!_i2c.writeRegs(_address, 0x81, &data, 1)) return false;

    // set gain to 16x (register 0x0F)
    data = 0x02; // 0 = 1x, 1 = 4x, 2 = 16x, 3 = 60x
    if(!_i2c.writeRegs(_address, 0x8F, &data, 1)) return false;

    // power on and enable ADCs (register 0x00)
    data = 0x03; // set bit 0 (activate internal oscillator) and bit 1 (enable ADCs)
    return _i2c.writeRegs(_address, 0x80, &data, 1);
}

bool TCS3472::readColor(void* color, bool includeC) {

    // read color data continously from the data registers (starting at 0x14 with clear or 0x16 with red)
    return _i2c.readRegs(_address, includeC ? 0xB4: 0xB6, color, includeC ? 8 : 6, true);
}