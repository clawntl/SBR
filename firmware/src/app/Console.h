// Text console over USB CDC (spec §8.3). Stage 3: ver, imuid, imu, stream, zero, peak, canping, canloop.
#pragma once

namespace Console {
// Prints the version banner.
void printBanner();
// Creates the console task on core 0.
void start();
}  // namespace Console
