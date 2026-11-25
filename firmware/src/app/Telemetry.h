// Telemetry task (spec §8.1), core 0. Text streams for IMU-Visualiser
// (`stream q`, Stage 2; `stream e`, Stage 3)
// and logging of IMU overruns from core 0 (spec §5.2).
#pragma once

namespace Telemetry {

enum class Stream { Off, Quat, Euler };

void start();

// `stream q|e|off`. Also switches the SFLP game vector in the FIFO (only for q).
bool setStream(Stream s);

}  // namespace Telemetry
