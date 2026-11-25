// Compile-time constants and version identity (spec §3).
#pragma once

#define FW_NAME    "SBR controller"
#define FW_VERSION "0.0.0"
#define FW_STAGE   0

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
