// Sensor-to-body axis map R_sb (spec §5.3): v_body = R_sb * v_sensor.
//
// R_sb is a signed permutation with determinant +1 (a proper rotation). Row i
// says which sensor axis is body axis i, the same as IMU-Visualiser's remap
// ("X = +Y" gives row 0 = {0, 1, 0}). For a quaternion the same map rotates
// the vector part and leaves w alone: that is IMU-Visualiser's similarity
// transform R_display = M R M^T.
#pragma once

#include <stdint.h>

using AxisMatrix = int8_t[3][3];

// True if m is a signed permutation matrix with determinant +1.
constexpr bool isProperAxisMap(const AxisMatrix &m)
{
    for (int i = 0; i < 3; ++i) {
        int rowNonZero = 0, colNonZero = 0;
        for (int j = 0; j < 3; ++j) {
            if (m[i][j] < -1 || m[i][j] > 1) return false;
            rowNonZero += m[i][j] != 0;
            colNonZero += m[j][i] != 0;
        }
        if (rowNonZero != 1 || colNonZero != 1) return false;
    }
    const int det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    return det == 1;
}

// out = m * in. `in` and `out` must not alias.
inline void applyAxisMap(const AxisMatrix &m, const float in[3], float out[3])
{
    for (int i = 0; i < 3; ++i) {
        out[i] = m[i][0] * in[0] + m[i][1] * in[1] + m[i][2] * in[2];
    }
}

// Quaternion (w, x, y, z) from sensor frame to body frame.
inline void applyAxisMapQuat(const AxisMatrix &m, const float in[4], float out[4])
{
    out[0] = in[0];
    applyAxisMap(m, &in[1], &out[1]);
}
