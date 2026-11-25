// CAN0 through ACAN-ESP32FD (spec §4.1). Stage 1: bus bring-up only.
//
// Test frame 0x7F0 (controller -> bus), first byte says what it is:
//   [0x01][seq u32 BE]                         ping, from `canping`
//   [0x02][id - 0x100][up to 6 data bytes]     echo of a received frame
//   [0x03][0xFF]                               `canloop` filter probe
// Only frames that pass the §4.1 filter (0x100-0x10F) are received, so the
// echo test sends from CAN Bench on 0x100-0x10F. `canloop` sends its test
// frames on 0x100-0x10F in loopback mode. Nothing here sends a current frame
// (0x010) or a driver command (0x02x).
#pragma once

#include <stdint.h>

namespace CanBus {

constexpr uint32_t kBitRate         = 1'000'000;
constexpr uint32_t kSamplePermill   = 875;
constexpr uint32_t kTestId          = 0x7F0;
constexpr uint32_t kPingIntervalMs  = 100;

// Starts CAN0 in normal mode (on core 1) and the CAN service task (core 0).
// Logs the begin() error code, actual bit rate and sample point. Returns
// true if the bus is up. A false return is fatal for motor use (spec §4.1).
bool begin();

bool isUp();

// Sends one ping frame. False if not queued.
bool ping();

// Periodic pings every 100 ms and echo of received frames.
void setTest(bool on);
bool testOn();

// `canloop`: restarts CAN0 in LOOP_BACK_NO_ACK, sends frames through the
// controller and the §4.1 filter, checks what comes back, then restarts in
// normal mode. Prints the result. Returns true on PASS.
bool loopbackSelfTest();

// Prints bit timing, error counters, FIFO state and test counters.
void printStatus();

}  // namespace CanBus
