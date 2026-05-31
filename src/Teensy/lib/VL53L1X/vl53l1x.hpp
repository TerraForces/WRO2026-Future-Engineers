/**
 * vl53l1x.hpp - highly optimized VL53L1X distance sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef VL53L1X_HPP
#define VL53L1X_HPP

// include I2C bus control interface
#include <i2c.hpp>

struct VL53L1X {
    VL53L1X(I2C *i2c, uint8_t address = 0x29);
    bool available();
    bool begin(bool longMode, uint8_t viewportCenter);
    bool getDistance(uint16_t* distance);
    bool setAddress(uint8_t address);
    bool setIntermeasurement(uint32_t ms);
    bool setROI(uint8_t w, uint8_t h, uint8_t c);
    bool setTimingBudget(uint32_t ms);
    bool start();
    bool stop();
private:
    I2C *_i2c;
    uint8_t _address;
    bool _longMode;
};

#endif