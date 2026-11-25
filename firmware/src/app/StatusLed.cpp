#include "app/StatusLed.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "PinConfig.h"
#include "util/ColourWheel.h"

namespace {

constexpr uint32_t kTickMs       = 10;   // BOOT sampling period
constexpr uint32_t kHueStepEvery = 3;    // ticks per hue step: 360 steps * 30 ms ~ 11 s per cycle
constexpr uint8_t  kLevel        = 32;   // LED brightness (of 255); the LED is bright and close

void statusTask(void *)
{
    // BOOT is a strapping pin. It's only read here, after boot.
    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

    bool stablePressed  = digitalRead(PIN_BOOT_BTN) == LOW;
    bool lastRawPressed = stablePressed;
    uint32_t hue        = 0;
    uint32_t tick       = 0;
    uint32_t pressCount = 0;

    for (;;) {
        if (tick % kHueStepEvery == 0) {
            const Rgb c = colourWheel(hue, kLevel);
            rgbLedWrite(PIN_LED_RGB, c.r, c.g, c.b);
            hue = (hue + 1) % 360;
        }

        // Debounce: two equal samples 10 ms apart.
        const bool rawPressed = digitalRead(PIN_BOOT_BTN) == LOW;
        if (rawPressed == lastRawPressed && rawPressed != stablePressed) {
            stablePressed = rawPressed;
            if (stablePressed) {
                ++pressCount;
                Serial.printf("BOOT pressed (%lu)\n", static_cast<unsigned long>(pressCount));
            } else {
                Serial.println("BOOT released");
            }
        }
        lastRawPressed = rawPressed;

        ++tick;
        vTaskDelay(pdMS_TO_TICKS(kTickMs));
    }
}

}  // namespace

void StatusLed::start()
{
    xTaskCreatePinnedToCore(statusTask, "status", 3072, nullptr, 1, nullptr, 0);
}
