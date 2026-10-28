/**
 * neopixel.hpp - NeoPixel multi-LED library
 * by TerraForces
 */

// prevent including multiple times
#ifndef NEOPIXEL_HPP
#define NEOPIXEL_HPP

// include integer types
#include <stdint.h>

// object representing a NeoPixel LED strip
struct NeoPixel {

    // 8-bit/channel RGB color in transfer order (green, red, blue)
    struct COLOR {
        uint8_t g, r, b;
    };

    /// @brief initializes NeoPixel object, no communication will be performed
    /// @param pin number of the data output pin
    NeoPixel(uint8_t pin);

    /// @brief sets a number of LEDs (starting at the beginning of the strip) to the same color
    /// @param color to be used for all LEDs
    /// @param count number of LEDs (starting at the beginning of the strip) to be modified
    /// @returns whether the command was sent to the LEDs (otherwise the cooldown of 300 us since the last write end is still active)
    bool fill(COLOR color, uint32_t count);

    /// @brief change the output pin
    /// @param pin new data output pin number
    void setPin(uint8_t pin);

    /// @brief writes color data to a number of LEDs (starting at the beginning of the strip)
    /// @param colors color data (containing min. count elements)
    /// @param count number of LEDs (starting at the beginning of the strip) to be modified
    /// @returns whether the command was sent to the LEDs (otherwise the cooldown of 300 us since the last write end is still active)
    bool show(COLOR* colors, uint32_t count);

private:
    volatile uint32_t *set, *clr; // pin state modification registers
    uint32_t msk;                 // pin bit mask
    uint32_t lastEndMicros;       // microsecond timestamp of the last write end (us since program start, overflow after around 71 mins 35 s)
};

#endif