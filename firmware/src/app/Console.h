// Text console over USB CDC (spec §8.3). Stage 2: ver, imuid, imu, stream, canping, canloop.
#pragma once

namespace Console {
// Prints the version banner.
void printBanner();
// Creates the console task on core 0.
void start();
}  // namespace Console
