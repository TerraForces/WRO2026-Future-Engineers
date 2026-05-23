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

// TCS3472x sensor object (works for all TCS3472 series color sensors)
struct TCS3472 {

    /// @brief initializes sensor object, no communication with the sensor will be performed
    /// @param i2c i2c bus to use for communication with the sensor
    /// @param address i2c address of the sensor (should always be 0x29)
    TCS3472(I2C *i2c, uint8_t address = 0x29);

    /// @brief starts sensor at 16x gain with an integration time of 2.4 ms (time between measurements: 4.8 ms)
    /// @returns whether the sensor was successfully configured
    bool begin();

    /// @brief reads latest color data from the sensor
    /// @param color pointer to memory where the color data should be written
    /// @param includeC include clear channel data (requires 8 byte buffer when enabled, 6 bytes when disabled)
    /// @returns whether color data has been read successfully
    bool readColor(void* color, bool includeC = true);

private:
    I2C *_i2c;
    uint8_t _address;
};

#endif