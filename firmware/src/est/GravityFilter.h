// Gravity filter (spec §6.1): Mahony-style filter on the unit up-vector u,
// with gyro bias b, at the IMU rate. No quaternion, no Euler angles, so no
// singularity anywhere. Lever-arm compensation predicts the non-gravity
// specific force at the IMU from the contact mode (§7.1).
//
// Body frame: x forward, y along the axle to the left, z up. Pure C++ (no
// Arduino), unit-tested on the host (§6.3).
#pragma once

#include <stdint.h>

#include "est/Vec3.h"

enum class ContactMode : uint8_t { Wheel, Ring };

struct GravityFilterConfig {
    float kp            = 1.0f;    // rad/s
    float ki            = 0.01f;   // rad/s^2
    float biasLimit     = 0.1f;    // rad/s, per axis
    float alphaCutoffHz = 40.0f;   // low-pass on the differentiated gyro
    float trustBandG    = 0.2f;    // accel trust falls to 0 at | |f| - g | = 0.2 g
    bool leverArm       = true;    // false: f_pred = 0 (for comparison)
    Vec3 rImu;                     // m, IMU from the axle centre, body axes
    float dC            = 0.0f;    // m, axle to ring centre along body +z
    float ringHalfWidth  = 0.0f;   // m, ring ellipse semi-axis along body x
    float ringHalfHeight = 0.0f;   // m, ring ellipse semi-axis along body z
    float ringCutRadius  = 0.0f;   // m, circle about the axle cut out of the rings
};

// Rolling at tilt theta, the ring centre sits h(theta) above the floor
// (the ring's support function) and, rolling without slipping, moves
// h * theta_dot forward and h' * theta_dot up. A circle (a = b = R) gives
// h = R, h' = h'' = 0.
struct RingSupport {
    float h, dh, d2h;  // h(theta), dh/dtheta, d2h/dtheta2
};

// Full ellipse, semi-axes a along body x and b along body z:
//   h = sqrt(a^2 sin^2 theta + b^2 cos^2 theta)
RingSupport ringSupport(float theta, float a, float b);

// The robot's rings: the ellipse (centre d_c above the axle) minus a circle of
// radius cutRadius about the axle, which removes the bottom of the ring near
// the wheels. Where the ellipse's lowest point falls in the cut, the ring
// rests on one of the two cut corners instead (it pivots there).
class RingShape {
public:
    RingShape() = default;
    RingShape(float a, float b, float dC, float cutRadius);

    RingSupport support(float theta) const;

    // Half-angle of the cut, in the ellipse parameter phi (0 = bottom of the
    // ring): the arc |phi| < phiCut is gone. 0 if the cut misses the ring.
    float cutHalfAngle() const { return phiCut_; }
    // Front cut corner relative to the ring centre, body x and z (the rear
    // one is its mirror image).
    float cornerX() const { return cornerX_; }
    float cornerZ() const { return cornerZ_; }

private:
    float a_ = 0.0f, b_ = 0.0f;
    float phiCut_ = 0.0f;
    float cornerX_ = 0.0f, cornerZ_ = 0.0f;
};

// Largest |theta| at which the wheels still touch the floor (spec §7.1):
// d_c cos theta + r_w >= h(theta). 0 if they don't even touch upright.
float wheelContactLimit(const RingShape &ring, float dC, float rW);

struct GravityInput {
    Vec3 gyro;          // rad/s, body axes
    Vec3 accel;         // m/s^2, specific force, body axes
    float dt;           // s
    ContactMode mode;
    float v    = 0.0f;  // wheel mode: ground speed from the wheels, m/s (0 until Stage 4)
    float vDot = 0.0f;  // its derivative, low-passed, m/s^2
};

class GravityFilter {
public:
    static constexpr float kG = 9.80665f;

    explicit GravityFilter(const GravityFilterConfig &cfg = GravityFilterConfig()) { setConfig(cfg); }

    void setConfig(const GravityFilterConfig &cfg)
    {
        cfg_  = cfg;
        ring_ = RingShape(cfg.ringHalfWidth, cfg.ringHalfHeight, cfg.dC, cfg.ringCutRadius);
    }
    const GravityFilterConfig &config() const { return cfg_; }

    // Next update() initialises u from the accelerometer; bias back to 0.
    void reset();
    // Starts from a given up-vector (tests).
    void reset(const Vec3 &u0);

    void update(const GravityInput &in);

    // Re-expresses the state after every input vector gets rotated by
    // rotateY(., angle) from now on (mounting offset change, `zero`).
    void rotateAboutY(float angle);

    void resetHeading() { psi_ = 0.0f; }

    bool initialised() const { return init_; }
    const Vec3 &up() const { return u_; }
    const Vec3 &bias() const { return b_; }
    const Vec3 &omega() const { return w_; }   // bias-corrected gyro
    const Vec3 &alpha() const { return alpha_; }
    const Vec3 &fPred() const { return fPred_; }
    float accelTrust() const { return trust_; }

    // Outputs (§6.2).
    float theta() const;      // atan2(-u_x, u_z), (-pi, pi]
    float thetaDot() const { return w_.y; }
    float lambda() const;     // asin(u_y)
    float psiDot() const { return dot(w_, u_); }
    float psi() const { return psi_; }
    bool poseValid() const;   // |u_y| <= sin 60 deg

private:
    Vec3 predictedForce(const GravityInput &in) const;

    GravityFilterConfig cfg_;
    RingShape ring_;
    bool init_ = false;
    Vec3 u_{0.0f, 0.0f, 1.0f};
    Vec3 b_;
    Vec3 w_;
    Vec3 wPrev_;
    bool havePrev_ = false;
    Vec3 alpha_;
    Vec3 fPred_;
    float trust_ = 0.0f;
    float psi_   = 0.0f;
};

// Contact mode from tilt with hysteresis (spec §7.1): wheels touch for
// |theta| <= thetaC.
ContactMode nextContactMode(ContactMode current, float theta, float thetaC, float hysteresis);
