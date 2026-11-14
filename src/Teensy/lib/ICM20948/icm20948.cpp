/**
 * icm20948.cpp - ICM-20948 9-axis sensor library with DMP support (over I2C)
 * by TerraForces
 */

// include header file
#include "icm20948.hpp"

uint8_t dmp_image[] = {
#include "icm20948_img.hpp"
};

#define ERROR(func, i) return (error << 16) | (func << 8) | i

ICM20948::ICM20948(I2C& i2c, uint8_t address) : _i2c(i2c), _address(address) {}

uint32_t ICM20948::begin() {
    uint32_t error = 0;

    if((error = _icm.begin(&_i2c, _address))) ERROR(func_begin, 0);
    if((error = _icm.initializeDMP())) ERROR(func_begin, 1);
    if((error = _icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION))) ERROR(func_begin, 2);

    // check id
    /*if((error = selectBank(0))) ERROR(func_begin, 0);
    uint8_t buf = 0;
    if(!read(0, &buf, 1)) ERROR(func_begin, 1);
    if(buf != 0xEA) ERROR(func_begin, 2);

    // software reset
    if(!read(6, &buf, 1)) ERROR(func_begin, 3);
    buf |= 0x80;
    if(!write(6, &buf, 1)) ERROR(func_begin, 4);
    delay(50);

    // disable sleep and low power mode
    if(!read(6, &buf, 1)) ERROR(func_begin, 5);
    buf &= 0x9F;
    if(!write(6, &buf, 1)) ERROR(func_begin, 6);

    // disable I2C passthrough
    if(!read(15, &buf, 1)) ERROR(func_begin, 7);
    buf &= 0xFD;
    if(!write(15, &buf, 1)) ERROR(func_begin, 8);

    // enable I2C master
    if((error = selectBank(3))) ERROR(func_begin, 9);
    if(!read(1, &buf, 1)) ERROR(func_begin, 10);
    buf |= 0x0F;
    if(!write(1, &buf, 1)) ERROR(func_begin, 11);
    if((error = selectBank(0))) ERROR(func_begin, 12);
    if(!read(3, &buf, 1)) ERROR(func_begin, 13);
    buf |= 0x20;
    if(!write(3, &buf, 1)) ERROR(func_begin, 14);

    // reset magnetometer
    buf = 0x01;
    if((error = magTransaction(true, 50, &buf))) ERROR(func_begin, 15);

    // try to detect the magnetometer
    for(uint8_t i = 0; i < 10; i++) {

        // check sensor id
        if((magTransaction(false, 0, &buf) == 0) && (buf == 0x48)) {
            if((magTransaction(false, 1, &buf) == 0) && (buf == 0x09)) break;
        }

        // failed to detect magnetometer
        if(i == 9) ERROR(func_begin, 16);

        // reset I2C master if not detected
        if((error = selectBank(0))) ERROR(func_begin, 17);
        if(!read(3, &buf, 1)) ERROR(func_begin, 18);
        buf |= 0x02;
        if(!write(3, &buf, 1)) ERROR(func_begin, 19);
        delay(10);
    }

    // configure communication to the magnetometer
    if((error = selectBank(3))) ERROR(func_begin, 20);
    buf = 0x8C;
    if(!write(3, &buf, 1)) ERROR(func_begin, 21);
    buf = 0x03;
    if(!write(4, &buf, 1)) ERROR(func_begin, 22);
    buf = 0xDA;
    if(!write(5, &buf, 1)) ERROR(func_begin, 23);
    buf = 0x0C;
    if(!write(3, &buf, 1)) ERROR(func_begin, 24);
    buf = 0x01;
    if(!write(6, &buf, 1)) ERROR(func_begin, 25);
    buf = 0x31;
    if(!write(4, &buf, 1)) ERROR(func_begin, 26);
    buf = 0x81;
    if(!write(5, &buf, 1)) ERROR(func_begin, 27);

    // Set the ODR configuration to 1100/2^4 = 68.75Hz
    buf = 0x04;
    if(!write(0, &buf, 1)) ERROR(func_begin, 28);
    
    // set clock source to auto
    if((error = selectBank(0))) ERROR(func_begin, 29);
    if(!read(6, &buf, 1)) ERROR(func_begin, 30);
    buf = (buf & 0xF8) | 0x01;
    if(!write(6, &buf, 1)) ERROR(func_begin, 31);
    buf = 0x40;
    if(!write(7, &buf, 1)) ERROR(func_begin, 32);

    // enable I2C master duty cycled mode
    if(!read(5, &buf, 1)) ERROR(func_begin, 33);
    buf |= 0x40;
    if(!write(5, &buf, 1)) ERROR(func_begin, 34);
    delay(1);

    // disable fifo
    if(!read(3, &buf, 1)) ERROR(func_begin, 35);
    buf &= 0xBF;
    if(!write(3, &buf, 1)) ERROR(func_begin, 36);

    // disable DMP
    if(!read(3, &buf, 1)) ERROR(func_begin, 37);
    buf &= 0x7F;
    if(!write(3, &buf, 1)) ERROR(func_begin, 38);

    // set acceleration full scale range to 4g
    if((error = selectBank(2))) ERROR(func_begin, 39);
    if(!read(20, &buf, 1)) ERROR(func_begin, 40);
    buf = (buf & 0xFB) | 0x02;
    if(!write(20, &buf, 1)) ERROR(func_begin, 41);

    // set gyro full scale range to 2000°/s
    if(!read(1, &buf, 1)) ERROR(func_begin, 42);
    buf |= 0x06;
    if(!write(1, &buf, 1)) ERROR(func_begin, 43);

    // enable gyro low pass filter
    if(!read(1, &buf, 1)) ERROR(func_begin, 44);
    buf |= 0x01;
    if(!write(1, &buf, 1)) ERROR(func_begin, 45);

    // disable fifo features
    if((error = selectBank(0))) ERROR(func_begin, 46);
    buf = 0;
    if(!write(102, &buf, 1)) ERROR(func_begin, 47);
    if(!write(103, &buf, 1)) ERROR(func_begin, 48);

    // disable raw data interrupts
    if(!read(17, &buf, 1)) ERROR(func_begin, 49);
    buf &= 0xFE;
    if(!write(17, &buf, 1)) ERROR(func_begin, 50);

    // reset fifo
    if(!read(104, &buf, 1)) ERROR(func_begin, 51);
    buf = 0x1F;
    if(!write(104, &buf, 1)) ERROR(func_begin, 52);
    buf = 0x1E;
    if(!write(104, &buf, 1)) ERROR(func_begin, 53);

    // set acceleration and gyro sample rate
    if((error = selectBank(2))) ERROR(func_begin, 54);
    buf = 19;
    if(!write(0, &buf, 1)) ERROR(func_begin, 55);
    if(!write(17, &buf, 1)) ERROR(func_begin, 56);
    buf = 0;
    if(!write(16, &buf, 1)) ERROR(func_begin, 57);

    // set DMP start address
    uint16_t dmpStart = 0x0010;
    if(!write(80, &dmpStart, 2)) ERROR(func_begin, 58);

    // write DMP firmware
    if((error = writeDMP(0, 0x90, dmp_image, sizeof(dmp_image)))) ERROR(func_begin, 59);

    // set DMP start address
    if(!write(80, &dmpStart, 2)) ERROR(func_begin, 62);

    // write 0x48 to hardware fix disable register
    if((error = selectBank(0))) ERROR(func_begin, 63);
    buf = 0x48;
    if(!write(117, &buf, 1)) ERROR(func_begin, 64);

    // select single fifo priority
    buf = 0xE4;
    if(!write(38, &buf, 1)) ERROR(func_begin, 65);

    // set acceleration scaling for the DMP
    uint32_t buf32 = 0x00000004u;
    if((error = writeDMP(1, 224, &buf32, 4))) ERROR(func_begin, 66);
    buf32 = 0x00000400;
    if((error = writeDMP(4, 244, &buf32, 4))) ERROR(func_begin, 67);

    // configure compass mounting matrix and DMP scaling
    uint32_t mountMultiplierZero  = 0x00000000;
    uint32_t mountMultiplierPlus  = 0x99999909;
    uint32_t mountMultiplierMinus = 0x676666F6;
    if((error = writeDMP(1, 112, &mountMultiplierPlus , 4))) ERROR(func_begin, 68);
    if((error = writeDMP(1, 116, &mountMultiplierZero , 4))) ERROR(func_begin, 69);
    if((error = writeDMP(1, 120, &mountMultiplierZero , 4))) ERROR(func_begin, 70);
    if((error = writeDMP(1, 124, &mountMultiplierZero , 4))) ERROR(func_begin, 71);
    if((error = writeDMP(1, 128, &mountMultiplierMinus, 4))) ERROR(func_begin, 72);
    if((error = writeDMP(1, 132, &mountMultiplierZero , 4))) ERROR(func_begin, 73);
    if((error = writeDMP(1, 136, &mountMultiplierZero , 4))) ERROR(func_begin, 74);
    if((error = writeDMP(1, 140, &mountMultiplierZero , 4))) ERROR(func_begin, 75);
    if((error = writeDMP(1, 144, &mountMultiplierMinus, 4))) ERROR(func_begin, 76);

    // configure B2S mounting matrix
    uint32_t b2sMountMultiplierZero = 0x00000000u;
    uint32_t b2sMountMultiplierPlus = 0x00000040u;
    if((error = writeDMP(13,  0, &b2sMountMultiplierPlus, 4))) ERROR(func_begin, 77);
    if((error = writeDMP(13,  4, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 78);
    if((error = writeDMP(13,  8, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 79);
    if((error = writeDMP(13, 12, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 80);
    if((error = writeDMP(13, 16, &b2sMountMultiplierPlus, 4))) ERROR(func_begin, 81);
    if((error = writeDMP(13, 20, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 82);
    if((error = writeDMP(13, 24, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 83);
    if((error = writeDMP(13, 28, &b2sMountMultiplierZero, 4))) ERROR(func_begin, 84);
    if((error = writeDMP(13, 32, &b2sMountMultiplierPlus, 4))) ERROR(func_begin, 85);

    // configure DMP gyro scaling factor
    if((error = selectBank(1))) ERROR(func_begin, 86);
    if(!read(40, &buf, 1)) ERROR(func_begin, 87);
    uint64_t buf64 = 84623001899965120ull / ((buf & 0x80) ? (1270 - (buf & 0x7F)) : (1270 + buf)) / 100000ull;
    buf32 = (buf64 > 0x7FFFFFFFu) ? 0xFFFFFF7Fu : __builtin_bswap32((uint32_t)buf64);
    if((error = writeDMP(1, 48, &buf32, 4))) ERROR(func_begin, 88);

    // configure gyro full scale range
    buf32 = 0x00000010;
    if((error = writeDMP(4, 140, &buf32, 4))) ERROR(func_begin, 89);

    // configure DMP acceleration parameters (depend on ODR)
    buf32 = 0x4992A403;
    if((error = writeDMP(1, 12, &buf32, 4))) ERROR(func_begin, 90);
    buf32 = 0x25499234;
    if((error = writeDMP(5, 176, &buf32, 4))) ERROR(func_begin, 91);
    buf32 = 0xDBB66D0B;
    if((error = writeDMP(5, 192, &buf32, 4))) ERROR(func_begin, 92);

    // configure acceleration calibration rate
    uint16_t buf16 = 0x0000;
    if((error = writeDMP(5, 228, &buf16, 2))) ERROR(func_begin, 93);

    // configure compass time buffer
    buf16 = 0x4500;
    if((error = writeDMP(7, 14, &buf16, 2))) ERROR(func_begin, 94);

    // enable DMP interrupt
    if((error = selectBank(0))) ERROR(func_begin, 95);
    if(!read(16, &buf, 1)) ERROR(func_begin, 96);
    buf |= 0x02;
    if(!write(16, &buf, 1)) ERROR(func_begin, 97);*/

    uint8_t buf = 0; uint16_t buf16 = 0;

    // disable sleep and low power mode
    if((error = selectBank(0))) ERROR(func_begin, 98);
    if(!read(6, &buf, 1)) ERROR(func_begin, 99);
    buf &= 0x9F;
    if(!write(6, &buf, 1)) ERROR(func_begin, 100);

    // enable DMP sensor (INV_ICM20948_SENSOR_ORIENTATION = ANDROID_SENSOR_ORIENTATION)
    /*buf16 = 0x0804;
    if((error = writeDMP(0, 64, &buf16, 2))) ERROR(func_begin, 101);
    buf16 = 0x0010;
    if((error = writeDMP(0, 66, &buf16, 2))) ERROR(func_begin, 102);
    buf16 = 0x0B00;
    if((error = writeDMP(0, 138, &buf16, 2))) ERROR(func_begin, 103);
    buf16 = 0xC003;
    if((error = writeDMP(0, 78, &buf16, 2))) ERROR(func_begin, 104);*/

    // set DMP sensor interval and counter
    buf16 = 0;
    if((error = writeDMP(0, 168, &buf16, 2))) ERROR(func_begin, 105);
    if((error = writeDMP(0, 136, &buf16, 2))) ERROR(func_begin, 106);
 
    // enable FIFO
    if((error = selectBank(0))) ERROR(func_begin, 107);
    if(!read(3, &buf, 1)) ERROR(func_begin, 108);
    buf |= 0x40;
    if(!write(3, &buf, 1)) ERROR(func_begin, 109);

    // enable DMP
    if(!read(3, &buf, 1)) ERROR(func_begin, 110);
    buf |= 0x80;
    if(!write(3, &buf, 1)) ERROR(func_begin, 111);

    // reset DMP
    if(!read(3, &buf, 1)) ERROR(func_begin, 112);
    buf |= 0x08;
    if(!write(3, &buf, 1)) ERROR(func_begin, 113);

    // reset fifo
    if(!read(104, &buf, 1)) ERROR(func_begin, 114);
    buf |= 0x1F;
    if(!write(104, &buf, 1)) ERROR(func_begin, 115);
    buf &= 0xFE;
    if(!write(104, &buf, 1)) ERROR(func_begin, 116);

    // wait until calibration is completed
    delay(10000);

    // reset fifo
    if(!read(104, &buf, 1)) ERROR(func_begin, 117);
    buf |= 0x1F;
    if(!write(104, &buf, 1)) ERROR(func_begin, 118);
    buf &= 0xFE;
    if(!write(104, &buf, 1)) ERROR(func_begin, 119);

    return 0;
}

uint32_t ICM20948::getRotation(float& rotation) {
    uint32_t error = 0;
 
    // read the size of fifo data
    uint16_t size;
    if((error = getFifoDataSize(size))) ERROR(func_getRotation, 0);

    // read the main header
    uint16_t header = 0;
    if(size < 2) ERROR(func_getRotation, 1);
    if((error = readFromFifo(&header, 2))) ERROR(func_getRotation, 2);
    size -= 2;
    header = __builtin_bswap16(header);

    // read header2 if available
    uint16_t header2 = 0;
    if((header & 0x0008) > 0) {
        if(size < 2) {
            if((error = getFifoDataSize(size))) ERROR(func_getRotation, 3);
            if(size < 2) ERROR(func_getRotation, 4);
        }
        if((error = readFromFifo(&header2, 2))) ERROR(func_getRotation, 5);
        header2 = __builtin_bswap16(header2);
        size -= 2;
    }

    // read buffer
    uint8_t fifoBytes[icm_20948_DMP_Maximum_Bytes];

    // macro to read "bytes" bytes from the fifo if the bit "mask" in "h" is set
    #define READ_FIFO(h, mask, bytes) \
    if(h & mask) {                                                               \
        if(size < bytes) {                                                       \
            if((error = getFifoDataSize(size))) ERROR(func_getRotation, 6);      \
            if(size < bytes) ERROR(func_getRotation, 7);                         \
        }                                                                        \
        if((error = readFromFifo(fifoBytes, bytes))) ERROR(func_getRotation, 8); \
        size -= bytes;                                                           \
    }

    // read payload and discard data until reaching Quat9
    READ_FIFO(header, 0x8000,  6); // acceleration
    READ_FIFO(header, 0x4000, 12); // rotation change
    READ_FIFO(header, 0x2000,  6); // compass
    READ_FIFO(header, 0x1000,  8); // ALS
    READ_FIFO(header, 0x0800, 12); // Quat6

    // read Quat9 and calculate the z-axis euler rotation (yaw) from the big endian integer quaternion data
    bool validRotation = false;
    if((header & DMP_header_bitmap_Quat9) > 0) {
        if(size < 14) {
            if((error = getFifoDataSize(size))) ERROR(func_getRotation, 9);
            if(size < 14) ERROR(func_getRotation, 10);
        }
        if((error = readFromFifo(fifoBytes, 14))) ERROR(func_getRotation, 11);
        size -= 14;

        // extract signed integers from big endian data
        union {
            uint8_t Bytes[14];
            struct {
                int32_t q0;
                int32_t q1;
                int32_t q2;
                int16_t accuracy;
            } data;
        } Quat9;
        const int DMP_Quat9_Byte_Ordering[icm_20948_DMP_Quat9_Bytes] = { 3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 13, 12 };
        for (int i = 0; i < 14; i++) Quat9.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i];
        
        // convert quaternion to float (divide by 2^30)
        float q0 = Quat9.data.q0 / 1073741824.0f;
        float q1 = Quat9.data.q1 / 1073741824.0f;
        float q2 = Quat9.data.q2 / 1073741824.0f;

        // calculate z-axis euler rotation (yaw)
        float temp = atan2(2.0f * ((q1 * q0) - sqrt(1.0f - ((q0 * q0) + (q1 * q1) + (q2 * q2))) * q2), 1.0f - (2.0f * ((q0 * q0) + (q2 * q2)))) * (-180.0f / ((float)PI));
    
        // only update rotation if temp is a valid value (not nan)
        if(!isnan(temp)) {

            // count revolutions
            if((temp < -60.0f) && (_lastRotation > 60.0f)) _revolutions++;
            else if((temp > 60.0f) && (_lastRotation < -60.0f)) _revolutions--;
            _lastRotation = temp;

            // add revolutions to rotation value
            rotation = temp + (_revolutions * 360.0f);
            validRotation = true;
        }
    }

    // read remaining payload
    READ_FIFO(header,  0x0200,  6); // PQuat6
    READ_FIFO(header,  0x0100, 14); // GeoMag
    READ_FIFO(header,  0x0080,  6); // pressure
    READ_FIFO(header,  0x0020, 12); // compass calibration
    READ_FIFO(header,  0x0010,  4); // step detector
    READ_FIFO(header2, 0x4000,  2); // acceleration accuracy
    READ_FIFO(header2, 0x2000,  2); // gyro accuracy
    READ_FIFO(header2, 0x1000,  2); // compass accuracy
    READ_FIFO(header2, 0x0400,  2); // pickup
    READ_FIFO(header2, 0x0080,  6); // activity recognition
    READ_FIFO(header2, 0x0040,  2); // secondary on/off

    // read footer
    READ_FIFO(1, 1, 2);

    if(size) ERROR(func_getRotation, 12);
    return validRotation ? 0 : ((func_getRotation << 8) | 13);
}

uint32_t ICM20948::getFifoDataSize(uint16_t& size) {
    uint32_t error = 0;

    // select register bank 0
    if((error = selectBank(0))) ERROR(func_getFifoDataSize, 0);

    // read high byte (only 12:8)
    uint8_t buf = 0;
    if(!read(112, &buf, 1)) ERROR(func_getFifoDataSize, 1);
    size = ((uint16_t)(buf & 0x1F)) << 8;

    // combine with low byte
    if(!read(113, &buf, 1)) ERROR(func_getFifoDataSize, 2);
    size |= (uint16_t)buf;
    return 0;
}

uint32_t ICM20948::magTransaction(bool w, uint8_t reg, uint8_t* data) {
    uint32_t error = 0;
    if(!data) ERROR(func_magTransaction, 0);

    // select register bank 3 (I2C)
    if((error = selectBank(3))) ERROR(func_magTransaction, 1);

    // write device address
    uint8_t buf = w ? 0x0C : 0x8C;
    if(!write(19, &buf, 1)) ERROR(func_magTransaction, 2);

    // write register address
    if(!write(20, &reg, 1)) ERROR(func_magTransaction, 3);

    // write data payload
    if(w) if(!write(22, data, 1)) ERROR(func_magTransaction, 4);

    // start transaction
    buf = 0x80;
    if(!write(21, &buf, 1)) ERROR(func_magTransaction, 5);

    // wait for completation (check I2C master status register in bank 0)
    if((error = selectBank(0))) ERROR(func_magTransaction, 6);
    for(uint32_t i = 0; i < 1000; i++) {
        if(!read(23, &buf, 1)) ERROR(func_magTransaction, 7);
        if(buf & 0x40) break;
        if(i == 999) ERROR(func_magTransaction, 8);
    }
    if(buf & 0x10) ERROR(func_magTransaction, 9);

    // nothing left to do when writing
    if(w) return 0;

    // read response from register bank 3
    if((error = selectBank(3))) ERROR(func_magTransaction, 10);
    if(!read(23, data, 1)) ERROR(func_magTransaction, 11);
    return 0;
}

bool ICM20948::read(uint8_t reg, void* data, uint32_t size) {
    return _i2c.readRegs(_address, reg, data, size, 0);
}

uint32_t ICM20948::readFromFifo(void* buf, uint32_t size) {
    uint32_t error = 0;

    // select bank 0
    if((error = selectBank(0))) ERROR(func_readFromFifo, 0);

    // read a number of bytes from the fifo buffer
    if(!read(114, buf, size)) ERROR(func_readFromFifo, 1);
    return 0;
}

uint32_t ICM20948::selectBank(uint8_t bank) {
    uint32_t error = 0;

    // only 3 register banks are accessible to the user
    if(bank > 3) ERROR(func_selectBank, 0);

    // nothing to do when the right bank is already selected
    if(bank == _lastBank) return 0;

    // write the next bank to register 127
    uint8_t buf = bank << 4;
    if(!write(127, &buf, 1)) ERROR(func_selectBank, 1);
    _lastBank = bank;
    return 0;
}

bool ICM20948::write(uint8_t reg, void* data, uint32_t size) {
    return _i2c.writeRegs(_address, reg, data, size);
}

uint32_t ICM20948::writeDMP(uint8_t bank, uint8_t reg, void* data, uint32_t size) {
    uint32_t error = 0;
    if(!data) ERROR(func_writeDMP, 0);

    // select user bank 0 (all DMP memory control registers are located there)
    if((error = selectBank(0))) ERROR(func_writeDMP, 1);

    // select dmp bank if it's different to the previous one
    if(_lastDMPBank != bank) {
        if(!write(126, &bank, 1)) ERROR(func_writeDMP, 2);
        _lastDMPBank = bank;
    }

    while(size > 0) {

        // select dmp start register
        if(!write(124, &reg, 1)) ERROR(func_writeDMP, 3);

        // write data block (max. 16 bytes)
        uint32_t bsize = min(16u, min(size, 0x100u - (uint32_t)reg));
        if(!write(125, &reg, bsize)) ERROR(func_writeDMP, 4);
        reg += bsize;
        size -= bsize;

        // increment DMP bank
        if(size && (reg == 0)) {
            bank++;
            if(!write(126, &bank, 1)) ERROR(func_writeDMP, 5);
        }
    }
    return 0;
}

/**
 * temporary
 */
uint32_t ICM20948::enableDMPSensor() {
    /*ICM_20948_Status_e result = ICM_20948_Stat_Ok;

    uint16_t inv_event_control = 0; // Use this to store the value for MOTION_EVENT_CTL
    uint16_t data_rdy_status = 0;   // Use this to store the value for DATA_RDY_STATUS

    uint8_t androidSensor = sensor_type_2_android_sensor(INV_ICM20948_SENSOR_ORIENTATION); // Convert sensor from enum inv_icm20948_sensor to Android numbering

    if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
        return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported (TO DO: Support B2S etc)

    // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
    uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
    if (delta == 0xFFFF)
        return ICM_20948_Stat_SensorNotSupported; // Bail if the sensor is not supported

    // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_0 / _enabled_Android_1
    unsigned long androidSensorAsBitMask;
    if (androidSensor < 32) // Sensors 0-31
    {
        androidSensorAsBitMask = 1L << androidSensor;
        if (state == 0) // Should we disable the sensor?
        {
        pdev->_enabled_Android_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
        }
        else
        {
        pdev->_enabled_Android_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor
        }
    }
    else // Sensors 32-
    {
        androidSensorAsBitMask = 1L << (androidSensor - 32);
        if (state == 0) // Should we disable the sensor?
        {
        pdev->_enabled_Android_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
        }
        else
        {
        pdev->_enabled_Android_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor
        }
    }

    // Now we know androidSensor is valid, reconstruct the value for DATA_OUT_CTL1 from _enabled_Android_0 and _enabled_Android_0
    delta = 0; // Clear delta
    for (int i = 0; i < 32; i++)
    {
        androidSensorAsBitMask = 1L << i;
        if ((pdev->_enabled_Android_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) is enabled
        {
        delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
        }
        if ((pdev->_enabled_Android_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) is enabled
        {
        delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
        }
        // Also check which bits need to be set in the Data Ready Status and Motion Event Control registers
        // Compare to INV_NEEDS_ACCEL_MASK, INV_NEEDS_GYRO_MASK and INV_NEEDS_COMPASS_MASK
        // See issue #150 - thank you @dobodu
        if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK) > 0)
        || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK1) > 0))
        {
        data_rdy_status |= DMP_Data_ready_Accel;
        inv_event_control |= DMP_Motion_Event_Control_Accel_Calibr;
        }
        if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK) > 0)
        || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK1) > 0))
        {
        data_rdy_status |= DMP_Data_ready_Gyro;
        inv_event_control |= DMP_Motion_Event_Control_Gyro_Calibr;
        }
        if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK) > 0)
        || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK1) > 0))
        {
        data_rdy_status |= DMP_Data_ready_Secondary_Compass;
        inv_event_control |= DMP_Motion_Event_Control_Compass_Calibr;
        }
    }

    result = ICM_20948_sleep(pdev, false); // Make sure chip is awake
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    result = ICM_20948_low_power(pdev, false); // Make sure chip is not in low power state
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    // Check if Accel, Gyro/Gyro_Calibr or Compass_Calibr/Quat9/GeoMag/Compass are to be enabled. If they are then we need to request the accuracy data via header2.
    uint16_t delta2 = 0;
    if ((delta & DMP_Data_Output_Control_1_Accel) > 0)
    {
        delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
    }
    if (((delta & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Gyro) > 0))
    {
        delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
    }
    if (((delta & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Compass) > 0) || ((delta & DMP_Data_Output_Control_1_Quat9) > 0) || ((delta & DMP_Data_Output_Control_1_Geomag) > 0))
    {
        delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
    }
    // TO DO: Add DMP_Data_Output_Control_2_Pickup etc. if required

    // Write the sensor control bits into memory address DATA_OUT_CTL1
    unsigned char data_output_control_reg[2];
    data_output_control_reg[0] = (unsigned char)(delta >> 8);
    data_output_control_reg[1] = (unsigned char)(delta & 0xff);
    pdev->_dataOutCtl1 = delta; // Diagnostics
    result = inv_icm20948_write_mems(pdev, DATA_OUT_CTL1, 2, (const unsigned char *)&data_output_control_reg);
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    // Write the 'header2' sensor control bits into memory address DATA_OUT_CTL2
    data_output_control_reg[0] = (unsigned char)(delta2 >> 8);
    data_output_control_reg[1] = (unsigned char)(delta2 & 0xff);
    pdev->_dataOutCtl2 = delta2; // Diagnostics
    result = inv_icm20948_write_mems(pdev, DATA_OUT_CTL2, 2, (const unsigned char *)&data_output_control_reg);
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    // Set the DATA_RDY_STATUS register
    data_output_control_reg[0] = (unsigned char)(data_rdy_status >> 8);
    data_output_control_reg[1] = (unsigned char)(data_rdy_status & 0xff);
    pdev->_dataRdyStatus = data_rdy_status; // Diagnostics
    result = inv_icm20948_write_mems(pdev, DATA_RDY_STATUS, 2, (const unsigned char *)&data_output_control_reg);
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    // Check which extra bits need to be set in the Motion Event Control register
    if ((delta & DMP_Data_Output_Control_1_Quat9) > 0)
    {
        inv_event_control |= DMP_Motion_Event_Control_9axis;
    }
    if (((delta & DMP_Data_Output_Control_1_Step_Detector) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_0) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_1) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_2) > 0))
    {
        inv_event_control |= DMP_Motion_Event_Control_Pedometer_Interrupt;
    }
    if ((delta & DMP_Data_Output_Control_1_Geomag) > 0)
    {
        inv_event_control |= DMP_Motion_Event_Control_Geomag;
    }

    // Set the MOTION_EVENT_CTL register
    data_output_control_reg[0] = (unsigned char)(inv_event_control >> 8);
    data_output_control_reg[1] = (unsigned char)(inv_event_control & 0xff);
    pdev->_motionEventCtl = inv_event_control; // Diagnostics
    result = inv_icm20948_write_mems(pdev, MOTION_EVENT_CTL, 2, (const unsigned char *)&data_output_control_reg);
    if (result != ICM_20948_Stat_Ok)
    {
        return result;
    }

    result = ICM_20948_low_power(pdev, true); // Put chip into low power state
    if (result != ICM_20948_Stat_Ok)
        return result;

    return result;*/
    return 0;
}