/**
 * vl53l1x.hpp - highly optimized VL53L1X distance sensor library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "vl53l1x.hpp"

uint8_t vl53l1xConfiguration[91] = {
	0x00, // 0x002D
    0x01, // 0x002E
    0x01, // 0x002F
    0x01, // 0x0030
    0x02, // 0x0031
    0x00, // 0x0032
    0x02, // 0x0033
    0x08, // 0x0034
    0x00, // 0x0035
    0x08, // 0x0036
    0x10, // 0x0037
    0x01, // 0x0038
    0x01, // 0x0039
    0x00, // 0x003A
    0x00, // 0x003B
    0x00, // 0x003C
    0x00, // 0x003D
    0xFF, // 0x003E
    0x00, // 0x003F
    0x0F, // 0x0040
    0x00, // 0x0041
    0x00, // 0x0042
    0x00, // 0x0043
    0x00, // 0x0044
    0x00, // 0x0045
    0x20, // 0x0046
    0x0B, // 0x0047
    0x00, // 0x0048
    0x00, // 0x0049
    0x02, // 0x004A
    0x0A, // 0x004B
    0x21, // 0x004C
    0x00, // 0x004D
    0x00, // 0x004E
    0x05, // 0x004F
    0x00, // 0x0050
    0x00, // 0x0051
    0x00, // 0x0052
    0x00, // 0x0053
    0xc8, // 0x0054
    0x00, // 0x0055
    0x00, // 0x0056
    0x38, // 0x0057
    0xFF, // 0x0058
    0x01, // 0x0059
    0x00, // 0x005A
    0x08, // 0x005B
    0x00, // 0x005C
    0x00, // 0x005D
    0x01, // 0x005E
    0xDB, // 0x005F
    0x0F, // 0x0060
    0x01, // 0x0061
    0xF1, // 0x0062
    0x0D, // 0x0063
    0x01, // 0x0064
    0x68, // 0x0065
    0x00, // 0x0066
    0x80, // 0x0067
    0x08, // 0x0068
    0xB8, // 0x0069
    0x00, // 0x006A
    0x00, // 0x006B
    0x00, // 0x006C
	0x00, // 0x006D
    0x0F, // 0x006E
    0x89, // 0x006F
    0x00, // 0x0070
    0x00, // 0x0071
    0x00, // 0x0072
    0x00, // 0x0073
    0x00, // 0x0074
	0x00, // 0x0075
    0x00, // 0x0076
    0x01, // 0x0077
    0x0F, // 0x0078
    0x0D, // 0x0079
    0x0E, // 0x007A
    0x0E, // 0x007B
    0x00, // 0x007C
    0x00, // 0x007D
    0x02, // 0x007E
    0xC7, // 0x007F
    0xFF, // 0x0080
    0x9B, // 0x0081
    0x00, // 0x0082
    0x00, // 0x0083
    0x00, // 0x0084
    0x01, // 0x0085
    0x00, // 0x0086
    0x00, // 0x0087
};

// read 8-bit register from VL53L1X ToF sensor
uint8_t vl53l1xReadByte(I2C* _i2c, uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {                 // try sending register address up to 5 times
        _i2c->beginTransmission(add);                // start I2C transmission to sensor address
        _i2c->write(reg >> 8);                       // write first byte of register address
        _i2c->write(reg);                     // write second byte of register address
        if(_i2c->endTransmission(false) == 0) break; // end I2C transmission without sending stop command
    }
    _i2c->requestFrom(add, (uint8_t)1);              // request one byte from VL53L1X sensor
    return _i2c->read();                             // read data byte from receive buffer
}

// read 16-bit register from VL53L1X ToF sensor
uint16_t vl53l1xReadWord(I2C* _i2c, uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {                 // try sending register address up to 5 times
        _i2c->beginTransmission(add);                // start I2C transmission to sensor address
        _i2c->write(reg >> 8);                       // write first byte of register address
        _i2c->write(reg);                     // write second byte of register address
        if(_i2c->endTransmission(false) == 0) break; // end I2C transmission without sending stop command
    }
    _i2c->requestFrom(add, (uint8_t)2);              // request two bytes from VL53L1X sensor
    uint16_t firstByte = _i2c->read();               // read first byte from receive buffer
    return (firstByte << 8) | (uint8_t)_i2c->read(); // read second byte from receive buffer and combine to 16-bit value
}

// write to 8-bit register of VL53L1X sensor
bool vl53l1xWriteByte(I2C* _i2c, uint8_t add, uint16_t reg, uint8_t data) {
    _i2c->beginTransmission(add); // start I2C transmission to sensor address
    _i2c->write(reg >> 8);        // write first byte of register address
    _i2c->write(reg);      // write second byte of register address
    _i2c->write(data);            // write data to the register
    return _i2c->endTransmission(true) == 0;  // end transmission and send stop bit
}

// write to 16-bit register of VL53L1X sensor
bool vl53l1xWriteWord(I2C* _i2c, uint8_t add, uint16_t reg, uint16_t data) {
    _i2c->beginTransmission(add); // start I2C transmission to sensor address
    _i2c->write(reg >> 8);        // write first byte of register address
    _i2c->write(reg);      // write second byte of register address
    _i2c->write(data >> 8);       // write first byte of data to the register
    _i2c->write(data);     // write second byte of data to the register
    return _i2c->endTransmission(true) == 0;  // end transmission and send stop bit
}

// write to 32-bit register of VL53L1X sensor
bool vl53l1xWriteDWord(I2C* _i2c, uint8_t add, uint16_t reg, uint32_t data) {
    _i2c->beginTransmission(add);     // start I2C transmission to sensor address
    _i2c->write(reg >> 8);            // write first byte of register address
    _i2c->write(reg);          // write second byte of register address
    _i2c->write(data >> 24);          // write first byte of data to the register
    _i2c->write(data >> 16); // write second byte of data to the register
    _i2c->write(data >> 8);  // write third byte of data to the register
    _i2c->write(data);         // write fourth byte of data to the register
    return _i2c->endTransmission(true) == 0;      // end transmission and send stop bit
}

VL53L1X::VL53L1X(I2C *i2c, uint8_t address) : _i2c(i2c), _address(address) {}

bool VL53L1X::available() {
    return (vl53l1xReadByte(_i2c, _address, 0x0031) & 1) == (!((vl53l1xReadByte(_i2c, _address, 0x0030) >> 4) & 1));
}

bool VL53L1X::begin(bool longMode, uint8_t viewportCenter) {
    _longMode = longMode;
    for(uint8_t j = 0; j < 91; j++) vl53l1xWriteByte(_i2c, _address, 0x002D + j, vl53l1xConfiguration[j]);
    if(!vl53l1xWriteByte(_i2c, _address, 0x0086, 0x01)) return false;
    vl53l1xWriteByte(_i2c, _address, 0x0087, 0x40);
    do delay(1);
    while((vl53l1xReadByte(_i2c, _address, 0x0031) & 1) == ((vl53l1xReadByte(_i2c, _address, 0x0030) >> 4) & 1));
    vl53l1xWriteByte(_i2c, _address, 0x0086, 0x01);
    vl53l1xWriteByte(_i2c, _address, 0x0087, 0x00);
    vl53l1xWriteByte(_i2c, _address, 0x0008, 0x09);
    vl53l1xWriteByte(_i2c, _address, 0x000B, 0x00);
    vl53l1xWriteByte(_i2c, _address, 0x004B, 0x0A);
    vl53l1xWriteByte(_i2c, _address, 0x0060, longMode ? 0x0F : 0x07);
    vl53l1xWriteByte(_i2c, _address, 0x0063, longMode ? 0x0D : 0x05);
    vl53l1xWriteByte(_i2c, _address, 0x0069, longMode ? 0xB8 : 0x38);
    vl53l1xWriteWord(_i2c, _address, 0x0078, longMode ? 0x0F0D : 0x0705);
    vl53l1xWriteWord(_i2c, _address, 0x007A, longMode ? 0x0E0E : 0x0606);
    setTimingBudget(longMode ? 50 : 33);
    setIntermeasurement(longMode ? 50 : 33);
    vl53l1xWriteByte(_i2c, _address, 0x007F, viewportCenter);
    vl53l1xWriteByte(_i2c, _address, 0x0080, 0x55);
    delay(15);
    vl53l1xWriteByte(_i2c, _address, 0x0086, 0x01);
    vl53l1xWriteByte(_i2c, _address, 0x0087, 0x40);
    /*for(uint8_t i = 0; i < sizeof(vl53l1xConfiguration); i++) _i2c->writeDRegs(_address, 0x002D, vl53l1xConfiguration + i, 1);
    uint8_t temp8 = 0;
    temp8 = 0x01;
    _i2c->writeDRegs(_address, 0x0086, &temp8, 1);
    temp8 = 0x40;
    return _i2c->writeDRegs(_address, 0x0087, &temp8, 1);
    bool a = false;
    do {
        delay(1);
        _i2c->readDRegs(_address, 0x0031, &temp8, 1, false);
        a = temp8 & 1;
        _i2c->readDRegs(_address, 0x0030, &temp8, 1, false);
    } while(a == ((temp8 >> 4) & 1));
    temp8 = 0x01;
    _i2c->writeDRegs(_address, 0x0086, &temp8, 1);
    temp8 = 0x00;
    _i2c->writeDRegs(_address, 0x0087, &temp8, 1);
    temp8 = 0x09;
    _i2c->writeDRegs(_address, 0x0008, &temp8, 1);
    temp8 = 0x00;
    _i2c->writeDRegs(_address, 0x000B, &temp8, 1);
    temp8 = 0x0A;
    _i2c->writeDRegs(_address, 0x004B, &temp8, 1);
    temp8 = 0x0F;
    _i2c->writeDRegs(_address, 0x0060, &temp8, 1);
    temp8 = 0x0D;
    _i2c->writeDRegs(_address, 0x0063, &temp8, 1);
    temp8 = 0xB8;
    _i2c->writeDRegs(_address, 0x0069, &temp8, 1);
    uint16_t temp16 = 0x0F0D;
    _i2c->writeDRegs(_address, 0x0078, &temp16, 2);
    temp16 = 0x0E0E;
    _i2c->writeDRegs(_address, 0x007A, &temp16, 2);
    temp16 = 0x01AE;
    _i2c->writeDRegs(_address, 0x005E, &temp16, 2);
    temp16 = 0x01E8;
    _i2c->writeDRegs(_address, 0x0061, &temp16, 2);
    _i2c->readDRegs(_address, 0x00DE, &temp16, 2, false);
    uint32_t temp32 = (uint32_t)((temp16 & 0x03FF) * 64.5f);
    _i2c->writeDRegs(_address, 0x006C, &temp32, 4);
    temp8 = 199;
    _i2c->writeDRegs(_address, 0x007F, &temp8, 1);
    temp8 = 0x55;
    _i2c->writeDRegs(_address, 0x0080, &temp8, 1);
    temp8 = 0x01;
    _i2c->writeDRegs(_address, 0x0086, &temp8, 1);
    temp8 = 0x40;
    return _i2c->writeDRegs(_address, 0x0087, &temp8, 1);*/
    return true;
}

bool VL53L1X::getDistance(uint16_t* distance) {
    /*uint8_t data = 0x01;
    _i2c->writeDRegs(_address, 0x0086, &data, 1);
    return _i2c->readDRegs(_address, 0x0096, distance, 2, false);*/
    vl53l1xWriteByte(_i2c, _address, 0x0086, 0x01);
    *distance = vl53l1xReadWord(_i2c, _address, 0x0096);
    return true;
}

bool VL53L1X::setAddress(uint8_t address) {
    _i2c->writeDRegs(_address, 0x0001, &address, 1);
    _address = address;
    return true;
}

bool VL53L1X::setIntermeasurement(uint32_t ms) {
    return vl53l1xWriteDWord(_i2c, _address, 0x006C, (uint32_t)((vl53l1xReadWord(_i2c, _address, 0x00DE) & 0x03FF) * 1.075f * ms));
}

bool VL53L1X::setROI(uint8_t w, uint8_t h, uint8_t c) {
    if(w < 4) return false;
    if(h < 4) return false;
    if(w > 16) return false;
    if(h > 16) return false;
    vl53l1xWriteByte(_i2c, _address, 0x007F, c);
    vl53l1xWriteByte(_i2c, _address, 0x0080, ((w - 1) << 4) | (h - 1));
    return true;
}

bool VL53L1X::setTimingBudget(uint32_t ms) {
	if(_longMode) {
        if(ms == 20) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x001E);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x0022);
		}
		if(ms == 33) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x0060);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x006E);
		}
		if(ms == 50) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x00AD);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x00C6);
		}
		if(ms == 100) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x01CC);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x01EA);
		}
		if(ms == 200) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x02D9);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x02F8);
		}
		if(ms == 500) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x048F);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x04A4);
        }
    }
    else {
		if(ms == 15) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x001D);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x0027);
        }
		if(ms == 20) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x0051);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x006E);
        }
		if(ms == 33) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x00D6);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x006E);
        }
		if(ms == 50) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x01AE);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x01E8);
        }
		if(ms == 100) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x02E1);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x0388);
        }
		if(ms == 200) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x03E1);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x0496);
        }
		if(ms == 500) {
			vl53l1xWriteWord(_i2c, _address, 0x005E, 0x0591);
			vl53l1xWriteWord(_i2c, _address, 0x0061, 0x05C1);
        }
	}
    return true;
}

bool VL53L1X::start() {
    vl53l1xWriteByte(_i2c, _address, 0x0086, 0x01);
    vl53l1xWriteByte(_i2c, _address, 0x0087, 0x40);
    return true;
}

bool VL53L1X::stop() {
    return vl53l1xWriteByte(_i2c, _address, 0x0087, 0x00);
}