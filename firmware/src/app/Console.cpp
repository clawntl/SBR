#include "app/Console.h"

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "Config.h"

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

void handleLine(char *line)
{
    // Trim trailing whitespace and leading spaces.
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    if (strcmp(line, "ver") == 0) {
        Console::printBanner();
    } else {
        Serial.printf("unknown command: %s\n", line);
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
