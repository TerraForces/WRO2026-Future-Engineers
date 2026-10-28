/**
 * neopixel.cpp - NeoPixel multi-LED library
 * by TerraForces
 */

// include header file
#include "neopixel.hpp"

// include I/O manipulation registers, bit masks and macros
#include <wiring.h>

NeoPixel::NeoPixel(uint8_t pin) : lastEndMicros() { setPin(pin); }

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

    uint8_t *p = (uint8_t*)&color, *end = p + 3, pix, mask;
    uint32_t cyc = ARM_DWT_CYCCNT + (F_CPU_ACTUAL / 800000);
    for(uint32_t i = 0; i < count; i++) {
        while(p < end) {
            pix = *p++;
            for (mask = 0x80; mask; mask >>= 1) {
                while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000));
                cyc = ARM_DWT_CYCCNT;
                *set = msk;
                if (pix & mask) {
                    while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 1250000));
                } else {
                    while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 4000000));
                }
                *clr = msk;
            }
        }
        p -= sizeof(COLOR);
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

    uint8_t *p = (uint8_t*)colors, *end = p + (3 * count), pix, mask;
    uint32_t cyc = ARM_DWT_CYCCNT + (F_CPU_ACTUAL / 800000);
    while (p < end) {
        pix = *p++;
        for (mask = 0x80; mask; mask >>= 1) {
            while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000));
            cyc = ARM_DWT_CYCCNT;
            *set = msk;
            if (pix & mask) {
                while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 1250000));
            } else {
                while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 4000000));
            }
            *clr = msk;
        }
    }
    while (ARM_DWT_CYCCNT - cyc < (F_CPU_ACTUAL / 800000));
    interrupts();

    // record microsecond timestamp for cooldown
    lastEndMicros = micros();
    return true;
}