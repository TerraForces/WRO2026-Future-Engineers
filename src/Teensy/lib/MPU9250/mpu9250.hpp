/**
 * mpu9250.hpp - MPU9250 IMU + DMP library
 * by TerraForces
 */

// prevent including multiple times
#ifndef MPU9250_HPP
#define MPU9250_HPP

// include I2C bus control interface
#include <i2c.hpp>

struct MPU9250 {
    MPU9250(I2C& i2c, uint8_t address = 0x68);
    bool begin();
    bool getRotation(float& rotation);
private:
    I2C& _i2c;
    uint8_t _address;
    int32_t _rounds;
    float _lastRotation;
};

#endif