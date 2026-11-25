// Angle wrapping.
#pragma once

#include <math.h>

// Wraps to (-pi, pi].
inline float wrapPi(float a)
{
    constexpr float kPi    = 3.14159265358979f;
    constexpr float kTwoPi = 6.28318530717959f;
    a = fmodf(a + kPi, kTwoPi);  // (-2pi, 2pi)
    if (a <= 0.0f) a += kTwoPi;  // (0, 2pi]
    return a - kPi;              // (-pi, pi]
}
