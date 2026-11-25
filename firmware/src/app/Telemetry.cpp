#include "app/Telemetry.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "control/ControlTask.h"

namespace {

constexpr uint32_t kPeriodMs = 20;  // 50 Hz text stream (spec §8.3)

volatile Telemetry::Stream gStream = Telemetry::Stream::Off;

void telemetryTask(void *)
{
    TickType_t wake       = xTaskGetTickCount();
    uint32_t lastQuat     = 0;
    uint32_t lastOverruns = 0, lastLate = 0, lastBadTags = 0;
    uint32_t tick         = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));

        ControlTask::Snapshot s;
        if (!ControlTask::read(s)) continue;

        if (gStream == Telemetry::Stream::Quat && s.quatCount != lastQuat) {
            lastQuat = s.quatCount;
            // IMU-Visualiser quaternion mode: w;x;y;z
            Serial.printf("%.5f;%.5f;%.5f;%.5f\n", s.quat[0], s.quat[1], s.quat[2], s.quat[3]);
        }

        // Spec §5.2 step 6: overruns and late drains are logged from core 0,
        // at most once a second.
        if (++tick % (1000 / kPeriodMs) != 0) continue;
        if (s.imu.overruns != lastOverruns || s.imu.late != lastLate ||
            s.imu.badTags != lastBadTags) {
            Serial.printf("IMU: overruns %lu, late %lu, bad tags %lu\n",
                          static_cast<unsigned long>(s.imu.overruns),
                          static_cast<unsigned long>(s.imu.late),
                          static_cast<unsigned long>(s.imu.badTags));
            lastOverruns = s.imu.overruns;
            lastLate     = s.imu.late;
            lastBadTags  = s.imu.badTags;
        }
    }
}

}  // namespace

void Telemetry::start()
{
    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 2, nullptr, 0);
}

bool Telemetry::setStream(Stream s)
{
    // The game vector is only batched while it's being streamed (spec §8.3).
    if (!ControlTask::requestGameVector(s == Stream::Quat)) return false;
    gStream = s;
    return true;
}
