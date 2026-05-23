/**
 * mpu9250.hpp - highly optimized MPU9250 IMU + DMP library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef MPU9250_HPP
#define MPU9250_HPP

// include I2C bus control interface
#include <i2c.hpp>

struct MPU9250 {
    MPU9250(I2C& i2c, float filter, uint8_t address = 0x68);
    bool begin();
    bool getRotation(float* rotation);
private:
    I2C& _i2c;
    uint8_t _address;
    int32_t _rounds;
    float _lastRotation, _secondLastRotation, _filter;
};

#endif