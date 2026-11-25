#include "app/Console.h"

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "app/Settings.h"
#include "app/Telemetry.h"
#include "can/CanBus.h"
#include "control/ControlTask.h"
#include "imu/Lsm6dsv16x.h"

static_assert(ESP_ARDUINO_VERSION_MAJOR == FW_EXPECT_ARDUINO_MAJOR &&
                  ESP_ARDUINO_VERSION_MINOR == FW_EXPECT_ARDUINO_MINOR &&
                  ESP_ARDUINO_VERSION_PATCH == FW_EXPECT_ARDUINO_PATCH,
              "Arduino-ESP32 core is not the pinned version (see platformio.ini)");

namespace {

constexpr size_t kLineMax = 96;

const char *resetReasonName(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:  return "power-on";
        case ESP_RST_EXT:      return "external pin";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "panic";
        case ESP_RST_INT_WDT:  return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT:      return "other watchdog";
        case ESP_RST_DEEPSLEEP:return "deep sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO:     return "SDIO";
        case ESP_RST_USB:      return "USB";
        case ESP_RST_JTAG:     return "JTAG";
        default:               return "other";
    }
}

// `imuid [n]`: n WHO_AM_I reads (default 1); all must read 0x70.
void cmdImuId(const char *arg)
{
    long n = 1;
    if (*arg != '\0') {
        char *end = nullptr;
        n         = strtol(arg, &end, 10);
        if (*end != '\0' || n < 1 || n > 100000) {
            Serial.println("usage: imuid [n], n = 1..100000");
            return;
        }
    }
    // The control task owns the SPI bus, so it does the reads.
    ControlTask::WhoAmIResult r;
    if (!ControlTask::whoAmI(static_cast<uint32_t>(n), r)) {
        Serial.println("WHO_AM_I: no answer from the control task");
        return;
    }
    Serial.printf("WHO_AM_I: %lu/%ld correct (0x70), %lu wrong",
                  static_cast<unsigned long>(r.good), n, static_cast<unsigned long>(r.wrong));
    if (r.wrong) Serial.printf(" (last 0x%02X)", r.lastWrong);
    Serial.printf(", %lu SPI errors\n", static_cast<unsigned long>(r.spiErrors));
}

// `stream q` / `stream off` (spec §8.3).
void cmdStream(const char *arg)
{
    Telemetry::Stream s;
    if (strcmp(arg, "q") == 0) {
        s = Telemetry::Stream::Quat;
    } else if (strcmp(arg, "e") == 0) {
        s = Telemetry::Stream::Euler;
    } else if (strcmp(arg, "off") == 0) {
        s = Telemetry::Stream::Off;
    } else {
        Serial.println("usage: stream q|e|off");
        return;
    }
    if (!Telemetry::setStream(s)) Serial.println("stream: control task not running");
}

// `imu`: sample and cycle rates over 2 s, counters since boot, latest data.
void cmdImu()
{
    constexpr uint32_t kWindowMs = 2000;
    ControlTask::Snapshot a, b;
    if (!ControlTask::read(a)) {
        Serial.println("imu: control task not running");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(kWindowMs));
    ControlTask::read(b);

    const float dt = (b.tUs - a.tUs) * 1e-6f;
    const ControlTask::ImuCounters &c = b.imu;
    Serial.printf("IMU: INT1 %s, SPI bus %s\n", b.int1Ok ? "ok" : "NOT SET UP",
                  b.spiBusOk ? "held" : "NOT HELD");
    Serial.printf("  over %.2f s: %.1f cycles/s, %.1f samples/s (IMU ODR %.1f Hz from freq fine %d)\n",
                  dt, (c.cycles - a.imu.cycles) / dt, (c.pairs - a.imu.pairs) / dt,
                  Lsm6dsv16x::kOdrHz * (1.0f + 0.0013f * Lsm6dsv16x::freqFine()),
                  Lsm6dsv16x::freqFine());
    Serial.printf("  since boot (%lu s): cycles %lu, samples %lu, INT1 timeouts %lu\n",
                  static_cast<unsigned long>(b.tUs / 1000000), static_cast<unsigned long>(c.cycles),
                  static_cast<unsigned long>(c.pairs), static_cast<unsigned long>(c.intTimeouts));
    Serial.printf("  overruns %lu, late %lu, unpaired %lu, bad tags %lu, SPI errors %lu, "
                  "max FIFO level %u\n",
                  static_cast<unsigned long>(c.overruns), static_cast<unsigned long>(c.late),
                  static_cast<unsigned long>(c.unpaired), static_cast<unsigned long>(c.badTags),
                  static_cast<unsigned long>(c.spiErrors), c.maxLevel);
    constexpr float kRadToDeg = 57.29578f;
    Serial.printf("  accel  %+.3f %+.3f %+.3f g (|a| %.3f)\n", b.accel[0], b.accel[1],
                  b.accel[2],
                  sqrtf(b.accel[0] * b.accel[0] + b.accel[1] * b.accel[1] + b.accel[2] * b.accel[2]));
    Serial.printf("  gyro   %+.2f %+.2f %+.2f dps\n", b.gyro[0] * kRadToDeg, b.gyro[1] * kRadToDeg,
                  b.gyro[2] * kRadToDeg);
    Serial.printf("  SFLP gravity %+.3f %+.3f %+.3f g (%lu samples)\n", b.gravity[0],
                  b.gravity[1], b.gravity[2], static_cast<unsigned long>(b.gravityCount));
    const uint32_t dPairs = c.pairs - a.imu.pairs;
    Serial.printf("  IMU timestamp %.2f ticks/sample (expect 48.00), dt %.1f us, dt glitches %lu\n",
                  dPairs ? static_cast<float>(b.imuTs - a.imuTs) / dPairs : 0.0f,
                  b.est.dt * 1e6f, static_cast<unsigned long>(c.dtGlitches));

    const ControlTask::Estimate &e = b.est;
    Serial.printf("EST: theta %+.2f deg (no comp %+.2f, SFLP %+.2f), lean %+.2f deg, %s, %s\n",
                  e.theta * kRadToDeg, e.thetaNoComp * kRadToDeg, e.thetaSflp * kRadToDeg,
                  e.lambda * kRadToDeg, e.ringMode ? "ring mode" : "wheel mode",
                  e.poseValid ? "pose valid" : "POSE INVALID (axle near vertical)");
    Serial.printf("  wheels touch for |theta| < %.1f deg (+-%.0f deg hysteresis)\n",
                  e.thetaC * kRadToDeg, kModeHysteresisDeg);
    Serial.printf("  theta rate %+.2f dps, turn rate %+.2f dps, heading %+.1f deg\n",
                  e.thetaDot * kRadToDeg, e.psiDot * kRadToDeg, e.psi * kRadToDeg);
    Serial.printf("  gyro bias %+.3f %+.3f %+.3f dps, accel trust %.2f, mounting offset %+.2f deg\n",
                  e.bias[0] * kRadToDeg, e.bias[1] * kRadToDeg, e.bias[2] * kRadToDeg,
                  e.accelTrust, e.mountPitch * kRadToDeg);
}

// `zero`: mounting offset so theta = 0 now, saved to NVS (spec §8.3).
void cmdZero()
{
    // Refused unless Disabled: there is no balance state before Stage 5, so
    // the robot is always Disabled here.
    constexpr float kRadToDeg = 57.29578f;
    ControlTask::Snapshot before;
    if (!ControlTask::read(before)) {
        Serial.println("zero: control task not running");
        return;
    }
    float mount = 0.0f;
    if (!ControlTask::zero(mount)) {
        Serial.println("zero: no answer from the control task");
        return;
    }
    const bool saved = Settings::saveMountPitch(mount);
    Serial.printf("zero: theta was %+.2f deg; mounting offset %+.2f -> %+.2f deg, %s\n",
                  before.est.theta * kRadToDeg, before.est.mountPitch * kRadToDeg,
                  mount * kRadToDeg, saved ? "saved" : "NOT SAVED (NVS error)");
}

// `peak start` / `peak`: peak theta errors against the integrated gyro, for
// the swing and slide tests.
void cmdPeak(const char *arg)
{
    if (strcmp(arg, "start") == 0) {
        Serial.println(ControlTask::peakStart() ? "peak: started" : "peak: control task not running");
        return;
    }
    if (*arg != '\0') {
        Serial.println("usage: peak [start]");
        return;
    }
    ControlTask::Snapshot s;
    if (!ControlTask::read(s) || !s.peak.active) {
        Serial.println("peak: not started (peak start)");
        return;
    }
    constexpr float kRadToDeg = 57.29578f;
    const ControlTask::PeakStats &p = s.peak;
    Serial.printf("peak over %.1f s, swing +-%.1f deg: theta %.2f deg, no comp %.2f deg, "
                  "SFLP %.2f deg\n",
                  p.seconds, p.swing * kRadToDeg, p.comp * kRadToDeg, p.noComp * kRadToDeg,
                  p.sflp * kRadToDeg);
}

// `canping`: one ping on 0x7F0, then the bus status. `canping on|off`: ping
// every 100 ms and echo received frames.
void cmdCanPing(const char *arg)
{
    if (strcmp(arg, "on") == 0) {
        CanBus::setTest(true);
        Serial.println("canping on: 0x7F0 every 100 ms, echoing frames received on 0x100-0x10F");
    } else if (strcmp(arg, "off") == 0) {
        CanBus::setTest(false);
        CanBus::printStatus();
    } else if (*arg == '\0') {
        Serial.printf("ping %s\n", CanBus::ping() ? "queued" : "NOT queued");
        CanBus::printStatus();
    } else {
        Serial.println("usage: canping [on|off]");
    }
}

void handleLine(char *line)
{
    // Trim trailing whitespace and leading spaces.
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    // Split into command and argument.
    char *arg = line;
    while (*arg != '\0' && *arg != ' ' && *arg != '\t') ++arg;
    if (*arg != '\0') {
        *arg++ = '\0';
        while (*arg == ' ' || *arg == '\t') ++arg;
    }

    if (strcmp(line, "ver") == 0 && *arg == '\0') {
        Console::printBanner();
    } else if (strcmp(line, "imuid") == 0) {
        cmdImuId(arg);
    } else if (strcmp(line, "canping") == 0) {
        cmdCanPing(arg);
    } else if (strcmp(line, "canloop") == 0 && *arg == '\0') {
        CanBus::loopbackSelfTest();
    } else if (strcmp(line, "stream") == 0) {
        cmdStream(arg);
    } else if (strcmp(line, "imu") == 0 && *arg == '\0') {
        cmdImu();
    } else if (strcmp(line, "zero") == 0 && *arg == '\0') {
        cmdZero();
    } else if (strcmp(line, "peak") == 0) {
        cmdPeak(arg);
    } else {
        Serial.printf("unknown command: %s%s%s\n", line, *arg ? " " : "", arg);
        Serial.println("commands: ver, imuid [n], imu, stream q|e|off, zero, peak [start], "
                       "canping [on|off], canloop");
    }
}

void consoleTask(void *)
{
    char line[kLineMax];
    size_t len       = 0;
    bool overflowed  = false;

    for (;;) {
        while (Serial.available() > 0) {
            const int c = Serial.read();
            if (c == '\r' || c == '\n') {
                if (overflowed) {
                    Serial.println("line too long");
                } else {
                    line[len] = '\0';
                    handleLine(line);
                }
                len        = 0;
                overflowed = false;
            } else if (len < kLineMax - 1) {
                line[len++] = static_cast<char>(c);
            } else {
                overflowed = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}  // namespace

void Console::printBanner()
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    Serial.println();
    Serial.printf("%s %s (stage %d), git %s\n", FW_NAME, FW_VERSION, FW_STAGE, FW_GIT_REV);
    Serial.printf("  %s: Arduino-ESP32 %d.%d.%d, ESP-IDF %s\n", FW_PLATFORM,
                  ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                  ESP_ARDUINO_VERSION_PATCH, esp_get_idf_version());
    Serial.printf("  chip %s rev v%d.%d, %d cores, %lu MHz\n", ESP.getChipModel(),
                  chip.revision / 100, chip.revision % 100, chip.cores,
                  static_cast<unsigned long>(ESP.getCpuFreqMHz()));
    Serial.printf("  flash %lu MB, PSRAM %lu MB (%lu KB free), heap %lu KB free\n",
                  static_cast<unsigned long>(ESP.getFlashChipSize() >> 20),
                  static_cast<unsigned long>(ESP.getPsramSize() >> 20),
                  static_cast<unsigned long>(ESP.getFreePsram() >> 10),
                  static_cast<unsigned long>(ESP.getFreeHeap() >> 10));
    Serial.printf("  reset reason: %s\n", resetReasonName(esp_reset_reason()));
}

void Console::start()
{
    xTaskCreatePinnedToCore(consoleTask, "console", 4096, nullptr, 3, nullptr, 0);
}
