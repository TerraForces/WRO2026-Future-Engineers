/**
 * neopixel.cpp - NeoPixel multi-LED library
 * by TerraForces
 */

// include header file
#include "neopixel.hpp"

// include I/O manipulation registers, bit masks and macros
#include <wiring.h>

NeoPixel::NeoPixel(uint8_t pin) : lastEndMicros() { setPin(pin); }

// send a byte one bit at a time (starting with MSB)
#define SEND_BYTE(byte) { \
    uint8_t buf = byte; \
    for(uint8_t mask = 0x80; mask; mask >>= 1) { \
        while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000)); \
        cyc = ARM_DWT_CYCCNT; \
        *set = msk; \
        if (buf & mask) while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 1250000)); \
        else while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 4000000)); \
        *clr = msk; \
    } \
}

bool NeoPixel::fill(COLOR color, uint32_t count) {

    // nothing to do when count is 0
    if(count == 0) return true;

    // check cooldown with overflow protection (300 us after end of last transaction)
    uint32_t now = micros();
    if(lastEndMicros > now) lastEndMicros = now;
    if((now - lastEndMicros) < 300L) return false;

    // enable cycle counter
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    // cycle count after last operation
    uint32_t cyc = ARM_DWT_CYCCNT + (F_CPU_ACTUAL / 800000);

    // repeat the color transaction count times
    for(uint32_t i = 0; i < count; i++) {

        // transmit color bytes in the order green, red, blue (required by the sensor)
        SEND_BYTE(color.g);
        SEND_BYTE(color.r);
        SEND_BYTE(color.b);
    }
    while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000));
    interrupts();

    // record microsecond timestamp for cooldown
    lastEndMicros = micros();
    return true;
}

void NeoPixel::setPin(uint8_t pin) {

    // read registers and bit mask from the lookup table
    set = portSetRegister(pin);
    clr = portClearRegister(pin);
    msk = digitalPinToBitMask(pin);
}

bool NeoPixel::show(COLOR* colors, uint32_t count) {
    
    // nothing to do when count is 0
    if(count == 0) return true;

    // colors has to be a valid pointer
    if(!colors) return false;

    // check cooldown with overflow protection (300 us after end of last transaction)
    uint32_t now = micros();
    if(lastEndMicros > now) lastEndMicros = now;
    if((now - lastEndMicros) < 300L) return false;

    // enable cycle counter
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    // cycle count after last operation
    uint32_t cyc = ARM_DWT_CYCCNT + (F_CPU_ACTUAL / 800000);

    // transmit count color values
    for(COLOR* end = colors + count; colors < end; colors++) {

        // transmit color bytes in the order green, red, blue (required by the sensor)
        SEND_BYTE(colors->g);
        SEND_BYTE(colors->r);
        SEND_BYTE(colors->b);
    }
    while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000));
    interrupts();

    // record microsecond timestamp for cooldown
    lastEndMicros = micros();
    return true;
}