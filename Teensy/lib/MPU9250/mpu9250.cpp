/**
 * mpu9250.hpp - highly optimized MPU9250 IMU + DMP library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "mpu9250.hpp"

// include c library for the MPU9250 gyro and acceleration sensor
extern "C" {
    #include <inv_mpu.h>
    #include <inv_mpu_dmp_motion_driver.h>
}

MPU9250::MPU9250(I2C& i2c, float filter, uint8_t address) : _i2c(i2c), _address(address), _rounds(0), _lastRotation(0.0f), _secondLastRotation(0.0f), _filter(filter) {}

bool MPU9250::begin() {
    int_param_s int_param = {};
    mpu_init(&int_param);
    mpu_set_bypass(1);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_int_level(0);
	mpu_set_int_latched(1);
    set_int_enable(1);
    mpu_set_lpf(5);
    mpu_set_gyro_fsr(1000);
    mpu_set_sample_rate(1000);
	mpu_reset_fifo();
    dmp_load_motion_driver_firmware();
    dmp_enable_6x_lp_quat(1);
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT);
    dmp_set_fifo_rate(50);
    mpu_set_dmp_state(1);
    return true;
}

inline float qToFloat(long value) {
    return (value >> 30) + ((value & 0x3FFFFFFF) / (float)0x40000000);
}

bool MPU9250::getRotation(float* rotation) {
    short gyro[3], accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;
    while(!dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more));
    float a0 = qToFloat(quat[0]);
    float a1 = qToFloat(quat[1]);
    float a2 = qToFloat(quat[2]);
    float a3 = qToFloat(quat[3]);
    float degrees = atan2(2.0f * ((a1 * a2) - (a0 * a3)), (-2.0f * ((a2 * a2) + (a3 * a3))) + 1.0f) * 57.29577951308232f;
    if(degrees < 0.0f) degrees += 360.0f;
    //if(abs(degrees - _lastRotation) < _filter) {
        if((degrees < 90.0f) && (_lastRotation > 270.0f)) _rounds++;
        if((degrees > 270.0f) && (_lastRotation < 90.0f)) _rounds--;
        *rotation = -(degrees + (_rounds * 360.0f));
        _lastRotation = degrees;
    //}
    return true;
}