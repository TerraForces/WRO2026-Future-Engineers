/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for EwokPlus25 Platform Layer
 *         RANGING SENSOR VERSION
 *
 */

#include <stdio.h>      // sprintf(), vsnprintf(), printf()
#include <stdint.h>
#include <string.h>     // strncpy(), strnlen()

#include "vl53l1_platform.hpp"
#include "vl53l1_platform_log.hpp"
#include <i2c.hpp>

#define VL53L1_get_register_name(a,b)

// flag to indicate if power board has been accessed
const uint32_t _power_board_in_use = 0;

// flag to indicate if we can use the extended voltage ranges (not laser safe!)
uint32_t _power_board_extended = 0;

// cache of the comms type flag
uint8_t global_comms_type = 0;

/* custom implementation start */

VL53L1_Error VL53L1_CommsInitialise(VL53L1_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz) {
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev) {
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	pdev->i2c->beginTransmission(pdev->i2c_slave_address);          // start I2C transmission to sensor address
    pdev->i2c->write(index >> 8);                                   // write first byte of register address
    pdev->i2c->write(index);                                        // write second byte of register address
	for(uint32_t i = 0; i < count; i++) pdev->i2c->write(pdata[i]); // write count bytes of payload
    return pdev->i2c->endTransmission(true) ? VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	for(uint8_t i = 0; i < 5; i++) {                                // try sending register address up to 5 times
        pdev->i2c->beginTransmission(pdev->i2c_slave_address);      // start I2C transmission to sensor address
        pdev->i2c->write(index >> 8);                               // write first byte of register address
        pdev->i2c->write(index);                                    // write second byte of register address
        if(pdev->i2c->endTransmission(false) == 0) break;           // end I2C transmission without sending stop command
    }
	if(pdev->i2c->requestFrom(pdev->i2c_slave_address, count) < count);// return VL53L1_ERROR_CONTROL_INTERFACE;
    for(uint32_t i = 0; i < count; i++) pdata[i] = pdev->i2c->read();
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data) {
	return VL53L1_WriteMulti(pdev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data) {
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[2];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0xFF);

	status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);

	return status;
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[4];

	// Split 32-bit word into MS ... LS bytes
	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data &  0x000000FF);

	status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_DWORD);

	return status;
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53L1_ReadMulti(pdev, index, buffer, 1);

	*pdata = buffer[0];

	return status;
}


VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	uint8_t buffer[2] = {};

	VL53L1_Error status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) | (uint16_t)buffer[1]);

	return status;
}

VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us) {
	delayMicroseconds(wait_us);
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms) {
	delay(wait_ms);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz) {
	*ptimer_freq_hz = 0;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count) {
	*ptimer_count = 0;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioXshutdown(uint8_t value) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioPowerEnable(uint8_t value) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptEnable(void (*function)(), uint8_t edge_type) {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptDisable() {
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms) {
	*ptick_count_ms = millis();
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms) {
	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53L1_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53L1_TRACE_FUNCTION_NONE);

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		/*if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);
		*/

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	/* Restore function logging */
	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

