// Telemetry task (spec §8.1), core 0. Stage 2: the `stream q` text stream
// for IMU-Visualiser, and logging of IMU overruns from core 0 (spec §5.2).
#pragma once

namespace Telemetry {

enum class Stream { Off, Quat };

void start();

// `stream q` / `stream off`. Also switches the SFLP game vector in the FIFO.
bool setStream(Stream s);

}  // namespace Telemetry
