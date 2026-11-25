// Task creation only (spec §8.2 rule 4): setup() starts the tasks and deletes
// itself, so Arduino's loop() never runs on core 1.
#include <Arduino.h>

#include "app/Console.h"
#include "app/StatusLed.h"

void setup()
{
    Serial.begin(115200);

    // USB CDC: give a connected terminal a moment so the banner isn't lost.
    // Doesn't wait at all if no host is attached.
    for (int i = 0; i < 50 && !Serial; ++i) delay(10);

    Console::printBanner();
    StatusLed::start();
    Console::start();

    vTaskDelete(nullptr);
}

void loop() {}
