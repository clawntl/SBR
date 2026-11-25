// Pin map (spec §2).
#pragma once
#include <driver/gpio.h>

// IMU, LSM6DSV16X on SPI (through the GPIO matrix; <= 10 MHz)
constexpr gpio_num_t PIN_IMU_CS   = GPIO_NUM_0;
constexpr gpio_num_t PIN_IMU_SCK  = GPIO_NUM_1;
constexpr gpio_num_t PIN_IMU_MOSI = GPIO_NUM_2;   // IMU SDI
constexpr gpio_num_t PIN_IMU_MISO = GPIO_NUM_3;   // IMU SDO
constexpr gpio_num_t PIN_IMU_INT1 = GPIO_NUM_46;  // FIFO watermark, rising edge

// CAN0 to the transceiver
constexpr gpio_num_t PIN_CAN_TX   = GPIO_NUM_4;
constexpr gpio_num_t PIN_CAN_RX   = GPIO_NUM_5;

// On the Zero itself.
// Named PIN_LED_RGB, not PIN_RGB_LED as in the spec: the Arduino esp32s31
// variant already #defines PIN_RGB_LED (as 60), which would break the
// declaration. The static_assert below ties the two together.
constexpr gpio_num_t PIN_LED_RGB  = GPIO_NUM_60;  // WS2812B-1010 (strapping pin: output only after boot)
constexpr gpio_num_t PIN_BOOT_BTN = GPIO_NUM_61;  // active low (strapping pin: don't hold at reset)

#ifdef PIN_RGB_LED
static_assert(PIN_LED_RGB == PIN_RGB_LED, "Zero RGB LED pin disagrees with the Arduino variant");
#endif

// Spare / optional (unused until the user decides; see §12)
// GPIO48: spare.
// GPIO50 / GPIO51: I2C SDA / SCL, reserved for a future INA226 battery monitor.
// GPIO47 / GPIO49: spare; CAN1 TX / RX if a second bus is ever used.
// GPIO6: spare, slow signals only (next to the antenna). GPIO58/59: UART0, leave free.
