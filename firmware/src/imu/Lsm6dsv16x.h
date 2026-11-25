// LSM6DSV16X SPI glue and configuration (spec §5.1).
//
// begin() and configure() run from setup(). From then on only the control
// task talks to the IMU (it holds the SPI bus): everything else asks it
// through ControlTask requests.
#pragma once

#include <stdint.h>

#include "lsm6dsv16x_reg.h"

namespace Lsm6dsv16x {

constexpr uint32_t kSpiClockHz = 10'000'000;  // datasheet maximum (spec §5.1)

// Configuration (spec §5.1).
constexpr float kOdrHz         = 960.0f;
constexpr float kAccelGPerLsb  = 0.000244f;              // +-8 g
constexpr float kGyroDpsPerLsb = 0.070f;                 // +-2000 dps
constexpr float kSflpGravMgPerLsb = 0.061f;              // SFLP gravity vector
constexpr uint8_t kFifoWatermarkWords = 2;               // 1 gyro + 1 accel
constexpr uint16_t kMaxWordsPerRead   = 9;               // 63 bytes: SPI FIFO is 64

// Sets up the SPI bus and device on core 1. Logs the result. False on an
// SPI driver error (it says nothing about whether an IMU answers).
bool begin();

// Reads WHO_AM_I (0x0F) through the ST driver. False on an SPI error.
bool readWhoAmI(uint8_t &id);

// Software reset and the §5.1 configuration: +-8 g, +-2000 dps, 960 Hz
// high-performance, SFLP at 480 Hz with only the gravity vector batched,
// FIFO in continuous mode with accel, gyro and timestamps, watermark 2,
// INT1 = FIFO threshold (push-pull, active high). Logs each failed step.
bool configure();

// INTERNAL_FREQ_FINE, read by configure(): the IMU's clock is off by
// 0.13 % per LSB, so its actual ODR is 960 * (1 + 0.0013 * freqFine).
int8_t freqFine();

// --- Control task only (it holds the SPI bus) ---

// Takes the SPI bus for good, so transactions skip the bus lock.
bool acquireBus();
bool readFifoStatus(lsm6dsv16x_fifo_status_t &status);
// Burst-reads `words` FIFO words (7 bytes each) into buf; words <= kMaxWordsPerRead.
bool readFifoWords(uint8_t *buf, uint16_t words);
// SFLP game rotation vector into the FIFO (for `stream q`) on or off.
// The gravity vector stays batched.
bool setGameVectorBatch(bool on);

// ST driver context (read/write callbacks bound to our SPI device).
const stmdev_ctx_t &ctx();

}  // namespace Lsm6dsv16x
