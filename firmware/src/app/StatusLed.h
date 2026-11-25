// Status LED and BOOT button (Stage 0: colour cycle, BOOT press logged).
// The supervisor task takes these over in Stage 4 (spec §8.1).
#pragma once

namespace StatusLed {
// Creates the status task on core 0.
void start();
}  // namespace StatusLed
