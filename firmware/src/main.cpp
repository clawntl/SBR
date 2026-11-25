// Task creation only (spec §8.2 rule 4): setup() starts the tasks and deletes
// itself, so Arduino's loop() never runs on core 1.
#include <Arduino.h>

#include "app/Console.h"
#include "app/Settings.h"
#include "app/StatusLed.h"
#include "app/Telemetry.h"
#include "can/CanBus.h"
#include "control/ControlTask.h"
#include "imu/Lsm6dsv16x.h"

void setup()
{
    Serial.begin(115200);

    // USB CDC: after a reset the Zero re-enumerates and a serial monitor takes
    // a few seconds to reattach, so wait for the port to be opened (HWCDC's
    // `Serial` is true only once the host polls it) before printing the boot
    // log. On USB with no terminal open this costs up to kTerminalWaitMs; with
    // no USB at all (no SOF after kUsbDetectMs) it stops waiting.
    constexpr uint32_t kTerminalWaitMs = 10000;
    constexpr uint32_t kUsbDetectMs    = 1000;
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < kTerminalWaitMs) {
        if (!Serial.isPlugged() && millis() - t0 > kUsbDetectMs) break;
        delay(10);
    }

    Console::printBanner();

    // IMU and CAN both install their interrupts on core 1 (spec §8.2 rule 1).
    if (Lsm6dsv16x::begin()) {
        uint8_t id    = 0;
        const bool ok = Lsm6dsv16x::readWhoAmI(id);
        Serial.printf("IMU: WHO_AM_I 0x%02X %s\n", id,
                      !ok ? "(SPI error)" : id == LSM6DSV16X_ID ? "ok" : "WRONG, expected 0x70");
        if (ok && id == LSM6DSV16X_ID && Lsm6dsv16x::configure()) {
            // From here only the control task talks to the IMU.
            const float mount = Settings::loadMountPitch();
            Serial.printf("EST: mounting offset %+.2f deg (from NVS)\n", mount * 57.29578f);
            if (!ControlTask::start(mount)) Serial.println("control task: start FAILED");
        } else {
            Serial.println("IMU: not configured; control task not started");
        }
    }
    CanBus::begin();

    Telemetry::start();
    StatusLed::start();
    Console::start();

    vTaskDelete(nullptr);
}

void loop() {}
