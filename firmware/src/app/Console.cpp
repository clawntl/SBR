#include "app/Console.h"

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "can/CanBus.h"
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
    long good = 0, wrong = 0, spiErr = 0;
    uint8_t id = 0, lastWrong = 0;
    for (long i = 0; i < n; ++i) {
        if (!Lsm6dsv16x::readWhoAmI(id)) {
            ++spiErr;
        } else if (id == LSM6DSV16X_ID) {
            ++good;
        } else {
            ++wrong;
            lastWrong = id;
        }
    }
    if (n == 1) {
        if (spiErr) {
            Serial.println("WHO_AM_I: SPI error");
        } else {
            Serial.printf("WHO_AM_I = 0x%02X (%s)\n", id,
                          id == LSM6DSV16X_ID ? "ok, LSM6DSV16X" : "WRONG, expected 0x70");
        }
        return;
    }
    Serial.printf("WHO_AM_I: %ld/%ld correct, %ld wrong", good, n, wrong);
    if (wrong) Serial.printf(" (last 0x%02X)", lastWrong);
    Serial.printf(", %ld SPI errors\n", spiErr);
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
    } else {
        Serial.printf("unknown command: %s%s%s\n", line, *arg ? " " : "", arg);
        Serial.println("commands: ver, imuid [n], canping [on|off], canloop");
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
