// Pin map (spec §2). Stage 0 has only the pins on the Zero itself; Stage 1
// adds the protoboard pins (IMU SPI, INT1, CAN, timing).
#pragma once
#include <driver/gpio.h>

// On the Zero itself.
// Named PIN_LED_RGB, not PIN_RGB_LED as in the spec: the Arduino esp32s31
// variant already #defines PIN_RGB_LED (as 60), which would break the
// declaration. The static_assert below ties the two together.
constexpr gpio_num_t PIN_LED_RGB  = GPIO_NUM_60;  // WS2812B-1010 (strapping pin: output only after boot)
constexpr gpio_num_t PIN_BOOT_BTN = GPIO_NUM_61;  // active low (strapping pin: don't hold at reset)

#ifdef PIN_RGB_LED
static_assert(PIN_LED_RGB == PIN_RGB_LED, "Zero RGB LED pin disagrees with the Arduino variant");
#endif
