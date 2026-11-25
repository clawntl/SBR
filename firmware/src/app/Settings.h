// Persistent settings in NVS (spec §3). Stage 3: only the mounting offset
// stored by `zero`. Plain key-value get/set/save comes in Stage 5.
//
// NVS writes stall both cores (spec §8.2 rule 2): callers refuse them unless
// the balance state is Disabled. There is no balance state before Stage 5, so
// the robot is always Disabled until then.
#pragma once

namespace Settings {

// Mounting offset about body y, rad (0 if never stored).
float loadMountPitch();
bool saveMountPitch(float rad);

}  // namespace Settings
