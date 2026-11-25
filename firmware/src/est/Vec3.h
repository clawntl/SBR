// Minimal 3-vector for the estimator. Header-only, no dependencies.
#pragma once

#include <math.h>

struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};

constexpr Vec3 operator*(float s, const Vec3 &v) { return v * s; }
constexpr float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float norm(const Vec3 &v) { return sqrtf(dot(v, v)); }

// Rotation about +y by angle a (right-handed): z towards x.
inline Vec3 rotateY(const Vec3 &v, float a)
{
    const float c = cosf(a), s = sinf(a);
    return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}
