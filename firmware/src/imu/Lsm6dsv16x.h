// LSM6DSV16X SPI glue (spec §5.1). Stage 1: SPI bus and WHO_AM_I only.
// Configuration, FIFO drain and the INT1 path come in Stage 2.
#pragma once

#include <stdint.h>

#include "lsm6dsv16x_reg.h"

namespace Lsm6dsv16x {

constexpr uint32_t kSpiClockHz = 10'000'000;  // datasheet maximum (spec §5.1)

// Sets up the SPI bus and device on core 1. Logs the result. False on an
// SPI driver error (it says nothing about whether an IMU answers).
bool begin();

// Reads WHO_AM_I (0x0F) through the ST driver. False on an SPI error.
bool readWhoAmI(uint8_t &id);

// ST driver context (read/write callbacks bound to our SPI device).
const stmdev_ctx_t &ctx();

}  // namespace Lsm6dsv16x
