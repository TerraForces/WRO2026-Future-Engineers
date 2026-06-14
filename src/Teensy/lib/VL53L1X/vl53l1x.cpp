/**
 * vl53l1x.hpp - highly optimized VL53L1X distance sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "vl53l1x.hpp"

VL53L1X::VL53L1X(I2C *i2c, uint8_t address) : _dev({ {}, address, i2c, 0 }) {}

bool VL53L1X::begin() {
    if(VL53L1_DataInit(&_dev) != VL53L1_ERROR_NONE) return false;
    if(VL53L1_StaticInit(&_dev) != VL53L1_ERROR_NONE) return false;
    if(!setTimingBudget(50)) return false;
    //if(VL53L1_SetXTalkCompensationEnable(&_dev, true) != VL53L1_ERROR_NONE) return false;
    return VL53L1_SetInterruptPolarity(&_dev, 1) == VL53L1_ERROR_NONE;
}

bool VL53L1X::getDistance(uint16_t* distance) {
    VL53L1_RangingMeasurementData_t measurement = {};
    if(VL53L1_GetRangingMeasurementData(&_dev, &measurement) != VL53L1_ERROR_NONE) return false;
    *distance = measurement.RangeMilliMeter;
    return VL53L1_WrByte(&_dev, 0x0086, 0x01) == VL53L1_ERROR_NONE;
}

bool VL53L1X::setAddress(uint8_t address) {
    if(VL53L1_SetDeviceAddress(&_dev, address << 1) != VL53L1_ERROR_NONE) return false;
    _dev.i2c_slave_address = address;
    return true;
}

bool VL53L1X::setROI(int8_t x, int8_t y, uint8_t w, uint8_t h) {
    if((x < -5) || (x > 5) || (y < -5) || (y > 5) || (w > 12) | (h > 12)) return false;
    VL53L1_UserRoi_t roi = {
        (uint8_t)(8 + x - (w >> 1)),
        (uint8_t)(7 + y + (h >> 1) + (h & 1)),
        (uint8_t)(7 + x + (w >> 1) + (w & 1)),
        (uint8_t)(8 + y - (h >> 1))
    };
    return VL53L1_SetUserROI(&_dev, &roi) == VL53L1_ERROR_NONE;
}

bool VL53L1X::setTimingBudget(uint32_t ms) {
    if(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&_dev, ms * 1000) != VL53L1_ERROR_NONE) return false;
    return VL53L1_SetInterMeasurementPeriodMilliSeconds(&_dev, ms + 4) == VL53L1_ERROR_NONE;
}

bool VL53L1X::start() {
    return VL53L1_StartMeasurement(&_dev) == VL53L1_ERROR_NONE;
}

bool VL53L1X::stop() {
    return VL53L1_StopMeasurement(&_dev) == VL53L1_ERROR_NONE;
}