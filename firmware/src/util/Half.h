// IEEE-754 half-precision (binary16) to float (spec §5.2). The SFLP game
// rotation vector comes out of the IMU FIFO as three half-floats.
#pragma once

#include <stdint.h>
#include <string.h>

inline float halfToFloat(uint16_t h)
{
    const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
    uint32_t exp        = (h >> 10) & 0x1Fu;
    uint32_t mant       = h & 0x3FFu;
    uint32_t bits;

    if (exp == 0) {
        if (mant == 0) {
            bits = sign;  // +-0
        } else {
            // Subnormal: normalise.
            exp = 127 - 15 + 1;
            while ((mant & 0x400u) == 0) {
                mant <<= 1;
                --exp;
            }
            mant &= 0x3FFu;
            bits = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 0x1F) {
        bits = sign | 0x7F800000u | (mant << 13);  // inf / NaN
    } else {
        bits = sign | ((exp + 127 - 15) << 23) | (mant << 13);
    }

    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}
