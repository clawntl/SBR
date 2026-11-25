// Compile-time constants and version identity (spec §3).
#pragma once

#define FW_NAME    "SBR controller"
#define FW_VERSION "0.2.0"
#define FW_STAGE   2

// Set by scripts/build_info.py from git. No build date or time anywhere in
// the image, so the same commit builds the same binary (Stage 0 acceptance).
#ifndef FW_GIT_REV
#define FW_GIT_REV "unknown"
#endif

// Pinned toolchain (platformio.ini). Printed in the banner; the build checks
// the core version so a platform change can't slip in unnoticed.
#define FW_PLATFORM "pioarduino 61.04.00"
#define FW_EXPECT_ARDUINO_MAJOR 4
#define FW_EXPECT_ARDUINO_MINOR 0
#define FW_EXPECT_ARDUINO_PATCH 0

#ifdef __cplusplus
#include "util/AxisMap.h"

// Sensor-to-body axis map R_sb (spec §5.3): v_body = R_sb * v_sensor.
// Body frame: x forward, y along the axle to the left, z up.
// Row i is the sensor axis that is body axis i, exactly as IMU-Visualiser's
// remap: "X = -Y" -> row 0 = {0, -1, 0}. Found in Stage 2; identity until then.
constexpr AxisMatrix kRsb = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};
static_assert(isProperAxisMap(kRsb), "kRsb must be a signed permutation with det +1");
#endif
