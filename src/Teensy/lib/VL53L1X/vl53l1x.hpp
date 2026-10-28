/**
 * vl53l1x.hpp - VL53L1X distance sensor library
 * by TerraForces
 */

// prevent including multiple times
#ifndef VL53L1X_HPP
#define VL53L1X_HPP

// include I2C bus control interface
#include <i2c.hpp>

// include modified official library
#include "lib/vl53l1_api.hpp"

// TCS3472x sensor object (works for all TCS3472 series color sensors)
struct VL53L1X {

    /// @brief initializes sensor object, no communication with the sensor will be performed
    /// @param i2c i2c bus to use for communication with the sensor
    /// @param address i2c address of the sensor (should always be 0x29 after power on)
    VL53L1X(I2C *i2c, uint8_t address = 0x29);

    /// @brief loads the firmware and configures the sensor
    /// @returns whether the setup was successful
    bool begin();

    /// @brief reads measured distance from the sensor
    /// @param distance pointer to memory which should receive the measurement in mm
    /// @returns whether the distance was successfully read and interrupt bit cleared
    bool getDistance(uint16_t* distance);

    /// @brief change the I2C address of the sensor
    /// @param address new I2C address - all future communication has to start with this address
    /// @returns whether the address was successfully changed
    bool setAddress(uint8_t address);

    /// @brief sets the region of interest (active region for measurements)
    /// @param x horizontal shift of the active region from the center
    /// @param y vertical shift of the active region from the center
    /// @param w width of the region in pixels
    /// @param h height of the region in pixels
    /// @returns whether the new range of interest was successfully set
    bool setROI(int8_t x, int8_t y, uint8_t w, uint8_t h);

    /// @brief sets the timing budget and intermeasurement period
    /// @param ms timing budget in milliseconds (intermeasurement period is set 4 ms longer)
    /// @returns whether timing budget and intermeasurement period were successfully set
    bool setTimingBudget(uint32_t ms);

    /// @brief starts continous measurements
    /// @returns whether the measurements were started successfully
    bool start();

    /// @brief stops continous measurements
    /// @returns whether the measurements were stopped successfully
    bool stop();

private:

    // VL53L1X device object
    VL53L1_Dev_t _dev;
};

#endif