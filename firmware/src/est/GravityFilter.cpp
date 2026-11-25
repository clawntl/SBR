#include "est/GravityFilter.h"

#include <math.h>

namespace {

constexpr float kTwoPi       = 6.28318530717959f;
constexpr float kSin60       = 0.866025403784f;
constexpr float kMinHorizNorm = 1e-3f;  // |y x u| below this: axle vertical

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline Vec3 normalized(const Vec3 &v)
{
    const float n = norm(v);
    return n > 0.0f ? v * (1.0f / n) : v;
}

}  // namespace

void GravityFilter::reset()
{
    init_     = false;
    b_        = Vec3();
    havePrev_ = false;
    alpha_    = Vec3();
    psi_      = 0.0f;
}

void GravityFilter::reset(const Vec3 &u0)
{
    reset();
    u_    = normalized(u0);
    init_ = true;
}

Vec3 GravityFilter::predictedForce(const GravityInput &in) const
{
    if (!cfg_.leverArm) return Vec3();

    // h: horizontal, perpendicular to the axle (forward when upright).
    const Vec3 yHat(0.0f, 1.0f, 0.0f);
    const Vec3 yxu = cross(yHat, u_);
    const float hn = norm(yxu);
    if (hn < kMinHorizNorm) return Vec3();  // axle vertical: skip the correction
    const Vec3 h = yxu * (1.0f / hn);

    Vec3 r, aRef;
    if (in.mode == ContactMode::Wheel) {
        // Reference point: the axle. Its acceleration along h, plus the
        // centripetal term when turning.
        r    = cfg_.rImu;
        aRef = in.vDot * h + (in.v * psiDot()) * cross(u_, h);
    } else {
        // Reference point: the ring centre. The elliptical rings roll without
        // slipping: the centre moves h*thd forward and h'*thd up, so its
        // acceleration is (h thdd + h' thd^2) forward + (h' thdd + h'' thd^2) up.
        r = cfg_.rImu - Vec3(0.0f, 0.0f, cfg_.dC);
        const RingSupport s = ring_.support(theta());
        const float thd = w_.y, thdd = alpha_.y;
        aRef = (s.h * thdd + s.dh * thd * thd) * h + (s.dh * thdd + s.d2h * thd * thd) * u_;
    }
    return aRef + cross(alpha_, r) + cross(w_, cross(w_, r));
}

void GravityFilter::update(const GravityInput &in)
{
    if (!init_) {
        // First sample: start from the measured up direction.
        const float n = norm(in.accel);
        u_    = n > 0.0f ? in.accel * (1.0f / n) : Vec3(0.0f, 0.0f, 1.0f);
        init_ = true;
    }
    const float dt = in.dt;

    // 1. Bias-corrected rate.
    w_ = in.gyro - b_;

    // Angular acceleration: differentiated gyro through a first-order low-pass.
    if (havePrev_ && dt > 0.0f) {
        const Vec3 raw = (w_ - wPrev_) * (1.0f / dt);
        const float k  = 1.0f - expf(-kTwoPi * cfg_.alphaCutoffHz * dt);
        alpha_ += (raw - alpha_) * k;
    }
    wPrev_    = w_;
    havePrev_ = true;

    // 2. Predicted non-gravity specific force at the IMU.
    fPred_ = predictedForce(in);

    // 3, 4. Measured up direction and how far to trust it.
    const Vec3 d   = in.accel - fPred_;
    const float dn = norm(d);
    Vec3 e;
    if (dn > 0.0f) {
        const Vec3 uMeas = d * (1.0f / dn);
        trust_ = clampf(1.0f - fabsf(dn - kG) / (cfg_.trustBandG * kG), 0.0f, 1.0f);
        // 5. Error and bias.
        e = cross(uMeas, u_);
    } else {
        trust_ = 0.0f;
    }

    const Vec3 we = e * trust_;
    const Vec3 wc = w_ + we * cfg_.kp;
    b_ -= we * (cfg_.ki * dt);
    b_.x = clampf(b_.x, -cfg_.biasLimit, cfg_.biasLimit);
    b_.y = clampf(b_.y, -cfg_.biasLimit, cfg_.biasLimit);
    b_.z = clampf(b_.z, -cfg_.biasLimit, cfg_.biasLimit);

    // 6. Propagate: a world-fixed vector seen from the rotating body.
    u_ = normalized(u_ - cross(wc, u_) * dt);

    psi_ += psiDot() * dt;
}

void GravityFilter::rotateAboutY(float angle)
{
    u_     = rotateY(u_, angle);
    b_     = rotateY(b_, angle);
    w_     = rotateY(w_, angle);
    wPrev_ = rotateY(wPrev_, angle);
    alpha_ = rotateY(alpha_, angle);
}

float GravityFilter::theta() const { return atan2f(-u_.x, u_.z); }

float GravityFilter::lambda() const { return asinf(clampf(u_.y, -1.0f, 1.0f)); }

bool GravityFilter::poseValid() const { return fabsf(u_.y) <= kSin60; }

RingSupport ringSupport(float theta, float a, float b)
{
    const float s = sinf(theta), c = cosf(theta);
    const float k = a * a - b * b;
    RingSupport r;
    r.h   = sqrtf(a * a * s * s + b * b * c * c);
    r.dh  = k * s * c / r.h;
    r.d2h = (k * (c * c - s * s) - r.dh * r.dh) / r.h;
    return r;
}

RingShape::RingShape(float a, float b, float dC, float cutRadius) : a_(a), b_(b)
{
    // Ellipse point at parameter phi, relative to the axle:
    //   (a sin phi, d_c - b cos phi); its distance from the axle equals the
    // cut radius where (b^2 - a^2) c^2 - 2 d_c b c + (a^2 + d_c^2 - r^2) = 0,
    // c = cos phi. The bottom (phi = 0) is b - d_c from the axle.
    if (cutRadius <= 0.0f || b - dC >= cutRadius) return;  // cut misses the ring
    const float A = b * b - a * a, B = -2.0f * dC * b, C = a * a + dC * dC - cutRadius * cutRadius;
    float c;
    if (fabsf(A) < 1e-12f) {
        c = -C / B;
    } else {
        const float disc = B * B - 4.0f * A * C;
        if (disc < 0.0f) return;
        // Root nearest the bottom: the larger c within [-1, 1].
        const float r1 = (-B - sqrtf(disc)) / (2.0f * A);
        const float r2 = (-B + sqrtf(disc)) / (2.0f * A);
        c = (r2 <= 1.0f) ? (r1 > r2 ? r1 : r2) : r1;
    }
    if (c < -1.0f || c > 1.0f) return;
    phiCut_  = acosf(c);
    cornerX_ = a * sinf(phiCut_);
    cornerZ_ = -b * c;
}

RingSupport RingShape::support(float theta) const
{
    // Where on the full ellipse the floor would touch: tan phi = (a/b) tan theta.
    const float phiStar = atan2f(a_ * sinf(theta), b_ * cosf(theta));
    if (fabsf(phiStar) >= phiCut_) return ringSupport(theta, a_, b_);

    // That point is cut away: the ring rests on the lower of the two corners.
    // A point p's support is p . down = p_x sin theta - p_z cos theta.
    const float s = sinf(theta), c = cosf(theta);
    const float x = (s >= 0.0f) ? cornerX_ : -cornerX_;
    RingSupport r;
    r.h   = x * s - cornerZ_ * c;
    r.dh  = x * c + cornerZ_ * s;
    r.d2h = -r.h;
    return r;
}

float wheelContactLimit(const RingShape &ring, float dC, float rW)
{
    // f > 0: wheels below the ring, i.e. touching.
    auto f = [&](float th) { return dC * cosf(th) + rW - ring.support(th).h; };
    constexpr float kHalfPi = 1.57079632679f;
    if (f(0.0f) <= 0.0f) return 0.0f;
    if (f(kHalfPi) > 0.0f) return kHalfPi;
    float lo = 0.0f, hi = kHalfPi;
    for (int i = 0; i < 40; ++i) {
        const float mid = 0.5f * (lo + hi);
        (f(mid) > 0.0f ? lo : hi) = mid;
    }
    return lo;
}

ContactMode nextContactMode(ContactMode current, float theta, float thetaC, float hysteresis)
{
    const float a = fabsf(theta);
    if (current == ContactMode::Wheel && a > thetaC + hysteresis) return ContactMode::Ring;
    if (current == ContactMode::Ring && a < thetaC - hysteresis) return ContactMode::Wheel;
    return current;
}
