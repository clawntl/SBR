// Hue to RGB for the status LED colour cycle. Pure, host-testable.
#pragma once
#include <stdint.h>

struct Rgb {
    uint8_t r, g, b;
};

// Full-saturation colour at `hue` degrees (any value; wrapped to [0, 360)),
// peak channel value `level`. Linear ramps between the six primaries and
// secondaries: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta.
inline Rgb colourWheel(uint32_t hue, uint8_t level)
{
    hue %= 360;
    const uint32_t sector = hue / 60;            // 0..5
    const uint32_t f      = hue % 60;            // 0..59 within the sector
    const uint8_t up      = static_cast<uint8_t>(level * f / 60);
    const uint8_t down    = static_cast<uint8_t>(level - up);
    switch (sector) {
        case 0:  return {level, up, 0};
        case 1:  return {down, level, 0};
        case 2:  return {0, level, up};
        case 3:  return {0, down, level};
        case 4:  return {up, 0, level};
        default: return {level, 0, down};
    }
}
