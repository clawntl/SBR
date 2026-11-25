// Control task (spec §8.1), Stage 2 skeleton: driven by IMU INT1, drains the
// FIFO and publishes the latest IMU data and counters. No estimator, no CAN.
//
// Out of the task: a seqlock snapshot. Into the task: a request queue polled
// without blocking at the start of each cycle (spec §8.2 rule 3).
#pragma once

#include <stdint.h>

namespace ControlTask {

struct ImuCounters {
    uint32_t cycles;       // wake-ups by INT1
    uint32_t intTimeouts;  // no INT1 for 3 ms: drained anyway
    uint32_t pairs;        // accel + gyro samples
    uint32_t overruns;     // FIFO overrun events
    uint32_t late;         // drains with more than 2 samples
    uint32_t unpaired;     // gyro or accel word without its partner
    uint32_t badTags;      // FIFO words with an unexpected tag
    uint32_t spiErrors;
    uint16_t maxLevel;     // highest FIFO level seen, words
};

struct Snapshot {
    bool int1Ok;          // GPIO ISR on INT1 installed
    bool spiBusOk;        // control task holds the SPI bus
    int64_t tUs;          // esp_timer time of the last cycle
    ImuCounters imu;
    uint32_t imuTs;       // IMU timestamp of the latest sample
    float gyro[3];        // body frame, rad/s
    float accel[3];       // body frame, g
    float gravity[3];     // SFLP gravity vector, body frame, g
    uint32_t gravityCount;
    float quat[4];        // SFLP game rotation (w, x, y, z), body frame
    uint32_t quatCount;
};

struct WhoAmIResult {
    uint32_t id;  // request id this answers
    uint32_t good, wrong, spiErrors;
    uint8_t lastWrong;
};

// Creates the control task on core 1. It installs the GPIO ISR service and
// the INT1 handler from there (spec §8.2 rule 1). Call after the IMU is
// configured.
bool start();

// Latest snapshot. False if none yet.
bool read(Snapshot &out);

// `stream q`: SFLP game rotation vector into the FIFO on or off.
bool requestGameVector(bool on);

// `imuid n`: n WHO_AM_I reads done by the control task (it owns SPI). Blocks
// the caller (console) until done or timeout. False on timeout.
bool whoAmI(uint32_t n, WhoAmIResult &out);

}  // namespace ControlTask
