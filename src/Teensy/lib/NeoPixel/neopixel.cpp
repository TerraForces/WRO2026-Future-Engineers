/**
 * neopixel.cpp - highly optimized NeoPixel multi-LED library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "neopixel.hpp"

// include I/O manipulation functions and macros
#include <wiring.h>

NeoPixel::NeoPixel(uint8_t pin) : lastEndMicros() { setPin(pin); }

bool NeoPixel::fill(COLOR color, uint32_t count) {
    if(!count) return true;
    uint32_t now = micros();
    if(lastEndMicros > now) lastEndMicros = now;
    if((now - lastEndMicros) < 300L) return false;
    uint8_t *p = (uint8_t*)&color, *end = p + 3, pix, mask;
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
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
    lastEndMicros = micros();
    return true;
}

void NeoPixel::setPin(uint8_t pin) {
    set = portSetRegister(pin);
    clr = portClearRegister(pin);
    msk = digitalPinToBitMask(pin);
}

bool NeoPixel::show(COLOR* colors, uint32_t count) {
    if(!count) return true;
    if(!colors) return false;
    uint32_t now = micros();
    if(lastEndMicros > now) lastEndMicros = now;
    if((now - lastEndMicros) < 300L) return false;
    uint8_t *p = (uint8_t*)colors, *end = p + (3 * count), pix, mask;
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
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
    lastEndMicros = micros();
    return true;
}