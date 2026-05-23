/**
 * neopixel.hpp - highly optimized NeoPixel multi-LED library
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef NEOPIXEL_HPP
#define NEOPIXEL_HPP

// include integer sizes
#include <stdint.h>

struct NeoPixel {
    struct COLOR {
        uint8_t g, r, b;
    };
    NeoPixel(uint8_t pin);
    bool fill(COLOR color, uint32_t count);
    void setPin(uint8_t pin);
    bool show(COLOR* colors, uint32_t count);

private:
    volatile uint32_t *set, *clr;
    uint32_t msk;
    uint32_t lastEndMicros;
};

#endif