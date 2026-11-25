// Compile-time constants and version identity (spec §3).
#pragma once

#define FW_NAME    "SBR controller"
#define FW_VERSION "0.3.0"
#define FW_STAGE   3

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
// remap: "X = -Y" -> row 0 = {0, -1, 0}.
// Stage 3 on the robot (2026-11-25): tilting forward read negative and the
// side lean was reversed, so the board is rotated 180 deg about z.
constexpr AxisMatrix kRsb = {
    {-1, 0, 0},
    {0, -1, 0},
    {0, 0, 1},
};
static_assert(isProperAxisMap(kRsb), "kRsb must be a signed permutation with det +1");
#endif

#ifdef __cplusplus
// Geometry from the user's CAD (spec §7.1, §12), 2026-11-25.
// Rings: ellipses, 150 mm tall (body z) x 135 mm wide (body x) when upright.
constexpr float kRingHalfWidthM    = 0.0675f;  // ellipse semi-axis along body x
constexpr float kRingHalfHeightM   = 0.075f;   // ellipse semi-axis along body z
// A circle about the axle is cut out of the rings to clear the wheels and
// allow more tilt on the wheels (may be enlarged later).
constexpr float kRingCutDiameterM  = 0.066f;
constexpr float kWheelRadiusM      = 0.029f;   // r_w (58 mm diameter, approximate)
constexpr float kAxleToRingCentreM = 0.048f;   // d_c, along body +z (ring centre above the axle)
// r_imu: IMU position from the axle centre, body axes (x fwd, y left, z up), m.
// Centred, 56.5 mm above the axle (8.5 mm above the ring centre).
constexpr float kImuPosM[3]        = {0.0f, 0.0f, 0.0565f};
// Upright the wheels reach 2 mm below the (uncut) ring ellipse; with the cut
// they touch the floor for |theta| < ~20.3 deg (computed at start-up, §7.1).
static_assert(kAxleToRingCentreM + kWheelRadiusM > kRingHalfHeightM,
              "wheels never protrude past the rings: check the geometry (spec §7.1)");
constexpr float kModeHysteresisDeg = 2.0f;   // contact-mode hysteresis (spec §7.1)

// Gravity filter (spec §6.1). k_i tuned in Stage 3: the spec's starting 0.01
// learns a gyro bias with tau ~ 100 s and fails the §6.3 bias test.
constexpr float kFilterKp            = 1.0f;    // rad/s
constexpr float kFilterKi            = 0.1f;    // rad/s^2
constexpr float kFilterAlphaCutoffHz = 40.0f;   // angular-acceleration low-pass
#endif
