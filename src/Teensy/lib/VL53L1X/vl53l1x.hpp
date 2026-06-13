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

// include official library
#include "lib/vl53l1_api.hpp"

// TCS3472x sensor object (works for all TCS3472 series color sensors)
struct VL53L1X {

    /// @brief initializes sensor object, no communication with the sensor will be performed
    /// @param i2c i2c bus to use for communication with the sensor
    /// @param address i2c address of the sensor (should always be 0x29 after poweron)
    VL53L1X(I2C *i2c, uint8_t address = 0x29);

    /// @brief starts and configures the sensor
    /// @returns whether the setup was successful
    bool begin();

    /// @brief reads measured distance from the sensor
    /// @param distance pointer to memory which should receive the measurement in mm
    /// @returns
    bool getDistance(uint16_t* distance);
    bool setAddress(uint8_t address);
    bool setROI(int8_t x, int8_t y, uint8_t w, uint8_t h);
    bool setTimingBudget(uint32_t ms);
    bool start();
    bool stop();
    VL53L1_Dev_t _dev;
};

#endif