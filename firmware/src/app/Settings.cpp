#include "app/Settings.h"

#include <Preferences.h>

namespace {
constexpr const char *kNamespace = "sbr";
constexpr const char *kMountPitch = "mountPitch";
}  // namespace

float Settings::loadMountPitch()
{
    Preferences p;
    if (!p.begin(kNamespace, true)) return 0.0f;  // read-only; fails if never written
    const float v = p.getFloat(kMountPitch, 0.0f);
    p.end();
    return v;
}

bool Settings::saveMountPitch(float rad)
{
    Preferences p;
    if (!p.begin(kNamespace, false)) return false;
    const bool ok = p.putFloat(kMountPitch, rad) == sizeof(float);
    p.end();
    return ok;
}
