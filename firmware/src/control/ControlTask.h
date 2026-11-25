// Control task (spec §8.1). Stage 3: driven by IMU INT1, drains the FIFO and
// runs the gravity filter (§6) on every sample. No controller, no CAN.
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
    uint32_t dtGlitches;   // timestamp gap outside 0.5-4.5 periods: nominal dt used
    uint16_t maxLevel;     // highest FIFO level seen, words
};

// Estimator outputs (spec §6.2), body frame, rad and rad/s.
struct Estimate {
    float theta, thetaDot, lambda, psi, psiDot;
    bool poseValid;
    bool ringMode;        // contact mode from tilt (spec §7.1)
    float thetaC;         // wheels touch the floor for |theta| < thetaC
    float bias[3];        // gyro bias, rad/s
    float accelTrust;     // 0..1
    float thetaNoComp;    // same filter without lever-arm compensation
    float thetaSflp;      // from the SFLP gravity vector (cross-check only)
    float mountPitch;     // mounting offset applied to all IMU vectors, rad
    float dt;             // last sample interval, s
};

// Peak errors against the integrated gyro since `peak start` (swing and
// slide tests, Stage 3). Each relative to its value at the start. rad.
struct PeakStats {
    bool active;
    float seconds;
    float swing;    // peak |theta_gyro - theta_gyro(0)|
    float comp;     // peak |theta - theta_gyro|
    float noComp;   // same without lever-arm compensation
    float sflp;     // same for theta_sflp
};

struct Snapshot {
    bool int1Ok;          // GPIO ISR on INT1 installed
    bool spiBusOk;        // control task holds the SPI bus
    int64_t tUs;          // esp_timer time of the last cycle
    ImuCounters imu;
    uint32_t imuTs;       // IMU timestamp of the latest sample
    float gyro[3];        // body frame, rad/s (mounting offset applied)
    float accel[3];       // body frame, g
    float gravity[3];     // SFLP gravity vector, body frame, g
    uint32_t gravityCount;
    float quat[4];        // SFLP game rotation (w, x, y, z), body frame (R_sb only)
    uint32_t quatCount;
    Estimate est;
    PeakStats peak;
    uint32_t zeroAck;     // id of the last `zero` request done
};

struct WhoAmIResult {
    uint32_t id;  // request id this answers
    uint32_t good, wrong, spiErrors;
    uint8_t lastWrong;
};

// Creates the control task on core 1 with the stored mounting offset. It
// installs the GPIO ISR service and the INT1 handler from there (spec §8.2
// rule 1). Call after the IMU is configured.
bool start(float mountPitch);

// Latest snapshot. False if none yet.
bool read(Snapshot &out);

// `stream q`: SFLP game rotation vector into the FIFO on or off.
bool requestGameVector(bool on);

// `imuid n`: n WHO_AM_I reads done by the control task (it owns SPI). Blocks
// the caller (console) until done or timeout. False on timeout.
bool whoAmI(uint32_t n, WhoAmIResult &out);

// `zero`: adds the current tilt to the mounting offset so theta = 0 now.
// Blocks until done; returns the new offset (rad). False on timeout.
bool zero(float &newMountPitch);

// `peak start`: restarts the peak-error window.
bool peakStart();

}  // namespace ControlTask
