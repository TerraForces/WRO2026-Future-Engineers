/**
 * i2c.hpp - I2C bus interface library (master mode only)
 * Wire library extended with sensor register access functions
 * by TerraForces
 */

// prevent including multiple times
#ifndef I2C_HPP
#define I2C_HPP

#include <Arduino.h>
#include <stdint.h>

#define I2C_BUFFER_LENGTH 255

struct I2C {
	typedef struct {
		const uint8_t pin;
		const uint32_t mux_val;
		volatile uint32_t* select_input_register;
		const uint32_t select_val;
	} pin_info_t;

	typedef struct {
		volatile uint32_t &clock_gate_register;
		uint32_t clock_gate_mask;
		pin_info_t sda_pins[2];
		pin_info_t scl_pins[2];
		IRQ_NUMBER_t irq_number;
		void (*irq_function)(void);
	} I2C_Hardware_t;

	static const I2C_Hardware_t i2c1_hardware;
	static const I2C_Hardware_t i2c3_hardware;
	static const I2C_Hardware_t i2c4_hardware;

    constexpr I2C(const uint32_t _portAddr, const I2C_Hardware_t &myhardware) : portAddr(_portAddr), hardware(myhardware) {}
    void setSDA(uint8_t pin);
	void setSCL(uint8_t pin);
    uint32_t write(uint8_t data);
	uint32_t write(const void *data, uint32_t size);
	int available() {
		return rxBufferLength - rxBufferIndex;
	}
	int read() {
		if (rxBufferIndex >= rxBufferLength) return -1;
		return rxBuffer[rxBufferIndex++];
	}
	int peek() {
		if (rxBufferIndex >= rxBufferLength) return -1;
		return rxBuffer[rxBufferIndex];
	}

	void begin(uint32_t frequency = 100000);
    void setClock(uint32_t frequency);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission(uint8_t sendStop = 1);
    uint8_t requestFrom(uint8_t address, uint32_t size, bool sendStop = true);
    
    /// @brief reads from registers of an I2C device (8-bit register address)
    /// @param address I2C address of the target device
    /// @param start register address where the read operation should start (most devices auto-increment this with every byte read)
    /// @param data buffer that should hold the data, has to hold min. size bytes
    /// @param size number of bytes that should be read
    /// @param sendStop whether a stop signal should be sent after transmitting the register address
    /// @returns whether size bytes have successfully been read
    bool readRegs(uint8_t address, uint8_t start, void* data, uint32_t size, bool sendStop);

    /// @brief reads from registers of an I2C device (16-bit register address)
    /// @param address I2C address of the target device
    /// @param start register address where the read operation should start (most devices auto-increment this with every byte read)
    /// @param data buffer that should hold the data, has to hold min. size bytes
    /// @param size number of bytes that should be read
    /// @param sendStop whether a stop signal should be sent after transmitting the register address
    /// @returns whether size bytes have successfully been read
    bool readDRegs(uint8_t address, uint16_t start, void* data, uint32_t size, bool sendStop);

    /// @brief writes to registers of an I2C device (8-bit register address)
    /// @param address I2C address of the target device
    /// @param start register address where the write operation should start (most devices auto-increment this with every byte read)
    /// @param data buffer containing the data that should be written
    /// @param size number of bytes that should be written
    /// @returns whether size bytes have successfully been written and acknowledged
    bool writeRegs(uint8_t address, uint8_t start, void* data, uint32_t size);

    /// @brief writes to registers of an I2C device (16-bit register address)
    /// @param address I2C address of the target device
    /// @param start register address where the write operation should start (most devices auto-increment this with every byte read)
    /// @param data buffer containing the data that should be written
    /// @param size number of bytes that should be written
    /// @returns whether size bytes have successfully been written and acknowledged
    bool writeDRegs(uint8_t address, uint16_t start, void* data, uint32_t size);

private:
	void isr(void);
	void configSDApin(uint8_t index);
	void configSCLpin(uint8_t index);
	bool wait_idle();
	bool force_clock();
	const uintptr_t portAddr;
	const I2C_Hardware_t &hardware;
	uint8_t	sda_pin_index_ = 0x0;	// default is always first item
	uint8_t	scl_pin_index_ = 0x0;
	uint8_t rxBuffer[I2C_BUFFER_LENGTH] = {};
	uint8_t rxBufferIndex = 0;
	uint8_t rxBufferLength = 0;
	uint8_t txAddress = 0;
	uint8_t txBuffer[I2C_BUFFER_LENGTH+1] = {};
	uint8_t txBufferIndex = 0;
	uint8_t txBufferLength = 0;
	uint8_t transmitting = 0;
	uint8_t slave_mode = 0;
	uint8_t irqcount = 0;
	uint8_t sda_pin_index = 0;
	uint8_t scl_pin_index = 0;
	void (*user_onRequest)(void) = nullptr;
	void (*user_onReceive)(int) = nullptr;
	friend void lpi2c1_isr(void);
	friend void lpi2c3_isr(void);
	friend void lpi2c4_isr(void);
	friend void sda_rising_isr0(void);
	friend void sda_rising_isr1(void);
};

extern I2C i2c0;
extern I2C i2c1;
extern I2C i2c2;

#endif