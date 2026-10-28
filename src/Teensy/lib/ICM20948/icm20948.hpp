/**
 * icm20948.hpp - ICM-20948 9-axis sensor library with DMP support (over I2C)
 * by TerraForces
 */

// prevent including multiple times
#ifndef ICM20948_HPP
#define ICM20948_HPP

// include I2C bus control interface
#include <i2c.hpp>

// ICM-20948 9-axis sensor object
struct ICM20948 {

    // function IDs - found in error codes
    enum FUNCTIONS {
        func_begin = 1,
        func_getRotation,
        func_getFifoDataSize,
        func_magTransaction,
        func_readFromFifo,
        func_selectBank,
        func_writeDMP
    };

    /// @brief initializes sensor object, no communication will be performed
    /// @param i2c I2C bus interface to use for communication with the sensor
    /// @param address I2C address to use for communication with the sensor (0x68 if AD0 is pulled low, 0x69 if AD0 is pulled high)
    ICM20948(I2C& i2c, uint8_t address = 0x68);

    /// @brief starts and configures the sensor
    /// @returns 0 if the sensor has been started and configured successfully, an error code otherwise
    uint32_t begin();

    /// @brief reads the quaternion values from the sensor and calculates the z-axis rotation relative to the starting position
    /// @param rotation the calculated z-axis rotation in degrees
    /// @returns 0 if the rotation was read successfully, an error code otherwise
    uint32_t getRotation(float& rotation);

private:

    // I2C bus interface for communication
    I2C& _i2c;

    // I2C address for communication
    uint8_t _address;

    // last rotation measurement
    float _lastRotation = 0.0f;

    // number of completed revolutions
    int32_t _revolutions = 0;

    // last selected register bank
    uint8_t _lastBank = 0xFF;

    // last selected DMP register bank
    uint8_t _lastDMPBank = 0;

    /// @brief reads the size of the data in the fifo buffer
    /// @param size fifo content size in bytes
    /// @returns 0 if the size was read successfully, an error code otherwise
    uint32_t getFifoDataSize(uint16_t& size);

    /// @brief transfers data from or to the magnetometer over the ICM-20948
    /// @param w true if data should be written, false if it should be read
    /// @param reg address of the register on the magnetometer that should be read or written to
    /// @param data pointer to data that should be written or read
    /// @returns 0 if the transaction has been completed successfully, an error code otherwise
    uint32_t magTransaction(bool w, uint8_t reg, uint8_t* data);

    /// @brief reads size bytes of data from the sensor starting at register reg
    /// @param reg starting register (multi-byte read behavior depends on the register)
    /// @param buf pointer to a buffer where the data should be read to (has to be min. size bytes large)
    /// @param size number of bytes that should be read
    /// @returns whether size bytes were successfully read
    bool read(uint8_t reg, void* data, uint32_t size);

    /// @brief reads a number of bytes from the fifo buffer
    /// @param buf pointer to a buffer where the data should be read to (has to be min. size bytes large)
    /// @param size number of bytes that should be read
    /// @returns 0 if size bytes have been successfully read from the fifo buffer, an error code otherwise
    uint32_t readFromFifo(void* buf, uint32_t size);

    /// @brief selects a user register bank for the following transfers
    /// @param bank the number of the register bank (only 0 - 3 are allowed)
    /// @returns 0 if the bank selection command has been written successfully or the bank is already selected, an error code otherwise
    uint32_t selectBank(uint8_t bank);

    /// @brief writes size bytes of data to the sensor starting at register reg
    /// @param reg starting register (multi-byte write behavior depends on the register)
    /// @param data pointer to data that should be written
    /// @param size number of bytes that should be written
    /// @returns whether size bytes were successfully written
    bool write(uint8_t reg, void* data, uint32_t size);

    /// @brief writes size bytes of data to DMP memory
    /// @param bank starting DMP memory register bank (increments on register overflow)
    /// @param reg starting register in DMP memory
    /// @param data pointer to data that should be written
    /// @param size number of bytes that should be written
    /// @returns 0 if size bytes were written successfully to DMP memory, an error code otherwise
    uint32_t writeDMP(uint8_t bank, uint8_t reg, void* data, uint32_t size);
};

#endif