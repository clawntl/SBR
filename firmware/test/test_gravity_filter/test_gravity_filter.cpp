// Host unit tests for the gravity filter (spec §6.3).
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "est/GravityFilter.h"
#include "util/Wrap.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt    = 1.0f / 960.0f;
constexpr float kPi    = 3.14159265358979f;
constexpr float kG     = GravityFilter::kG;
constexpr float kDeg   = kPi / 180.0f;

// The robot's geometry (Config.h): elliptical rings 135 x 150 mm.
constexpr float kRingA = 0.0675f;  // semi-axis along body x
constexpr float kRingB = 0.075f;   // semi-axis along body z
constexpr float kDc    = 0.048f;
constexpr float kRw    = 0.029f;
constexpr float kCutR  = 0.033f;  // 66 mm circle about the axle cut out of the rings
const Vec3 kRImu(0.0f, 0.0f, 0.0565f);

// Ring-centre height above the floor at tilt th, by brute force: the lowest
// of the remaining ring points (ellipse minus the cut), relative to the ring
// centre. Independent of RingShape.
double ringHeightBrute(double th, double cutR = kCutR, int n = 200000)
{
    const double a = kRingA, b = kRingB, dC = kDc;
    double best = -1e9;
    for (int i = 0; i < n; ++i) {
        const double phi = 2.0 * 3.141592653589793 * i / n;
        const double x = a * sin(phi), z = -b * cos(phi);       // from the ring centre
        if (x * x + (z + dC) * (z + dC) < cutR * cutR) continue;  // cut away (axle at z = -d_c)
        const double h = x * sin(th) - z * cos(th);               // along "down"
        if (h > best) best = h;
    }
    return best;
}

// Ring-centre height used for the rolling kinematics: the analytic h from
// RingShape (its derivatives are not used; the kinematics differentiate
// numerically).
const RingShape kRing(kRingA, kRingB, kDc, kCutR);
double ringHeight(double th) { return kRing.support(static_cast<float>(th)).h; }

// Ring-centre position in the world while rolling without slipping: it moves
// forward by the integral of h(theta) d theta and sits h(theta) high.
// Simpson's rule; independent of the filter's analytic derivatives.
void ringCentre(double th, double &x, double &z)
{
    const int n = 200;  // even
    const double step = th / n;
    double sum = ringHeight(0.0) + ringHeight(th);
    for (int i = 1; i < n; ++i) sum += (i % 2 ? 4.0 : 2.0) * ringHeight(i * step);
    x = sum * step / 3.0;
    z = ringHeight(th);
}

// Gains used in firmware (Config.h).
GravityFilterConfig config(bool leverArm = true)
{
    GravityFilterConfig c;
    c.kp         = 1.0f;
    c.ki         = 0.1f;
    c.leverArm   = leverArm;
    c.rImu           = kRImu;
    c.dC             = kDc;
    c.ringHalfWidth  = kRingA;
    c.ringHalfHeight = kRingB;
    c.ringCutRadius  = kCutR;
    return c;
}

// Up-vector in body axes for tilt theta and side lean lambda (spec §5.3).
Vec3 upFrom(float theta, float lambda)
{
    return {-sinf(theta) * cosf(lambda), sinf(lambda), cosf(theta) * cosf(lambda)};
}

GravityInput staticInput(const Vec3 &u, const Vec3 &gyro = Vec3())
{
    GravityInput in;
    in.gyro  = gyro;
    in.accel = u * kG;
    in.dt    = kDt;
    in.mode  = ContactMode::Wheel;
    return in;
}

// Body turning at a constant rate about the (fixed) axle: the IMU also feels
// the centripetal force at r_imu.
GravityInput rotatingInput(const Vec3 &u, const Vec3 &omega)
{
    GravityInput in = staticInput(u, omega);
    in.accel += cross(omega, cross(omega, kRImu));
    return in;
}

float degErr(float a, float b) { return fabsf(wrapPi(a - b)) / kDeg; }

// Body rotating about +y with theta(t). Specific force at the IMU for a
// fixed axle (wheel mode) or rolling on the rings (ring mode).
struct PitchMotion {
    float amp, freqHz;
    ContactMode mode;

    void at(float t, float &th, float &thd, float &thdd) const
    {
        double a, b, c;
        atD(t, a, b, c);
        th = static_cast<float>(a), thd = static_cast<float>(b), thdd = static_cast<float>(c);
    }
    void atD(double t, double &th, double &thd, double &thdd) const
    {
        const double w = 2.0 * 3.141592653589793 * freqHz;
        th   = amp * sin(w * t);
        thd  = amp * w * cos(w * t);
        thdd = -amp * w * w * sin(w * t);
    }

    GravityInput input(float t) const
    {
        float th, thd, thdd;
        at(t, th, thd, thdd);
        const Vec3 u     = upFrom(th, 0.0f);
        const Vec3 h     = {cosf(th), 0.0f, sinf(th)};  // world forward, body axes
        const Vec3 omega = {0.0f, thd, 0.0f};
        const Vec3 alpha = {0.0f, thdd, 0.0f};
        Vec3 r = kRImu, aRef;
        if (mode == ContactMode::Ring) {
            // World acceleration of the ring centre by central differences of
            // its position along the trajectory (fixed axle in wheel mode).
            r = kRImu - Vec3(0.0f, 0.0f, kDc);
            const double tau = 1e-3;
            double xm, zm, x0, z0, xp, zp, thm, th0, thp, unused1, unused2;
            atD(t - tau, thm, unused1, unused2);
            atD(t, th0, unused1, unused2);
            atD(t + tau, thp, unused1, unused2);
            ringCentre(thm, xm, zm);
            ringCentre(th0, x0, z0);
            ringCentre(thp, xp, zp);
            const float ax = static_cast<float>((xp - 2 * x0 + xm) / (tau * tau));
            const float az = static_cast<float>((zp - 2 * z0 + zm) / (tau * tau));
            aRef = ax * h + az * u;  // world x -> h, world z -> u in body axes
        }
        const Vec3 aImu = aRef + cross(alpha, r) + cross(omega, cross(omega, r));
        GravityInput in;
        in.gyro  = omega;
        in.accel = aImu + u * kG;
        in.dt    = kDt;
        in.mode  = mode;
        return in;
    }
};

// Peak |theta error| in degrees over [settle, duration].
float peakError(const PitchMotion &m, bool leverArm, float duration = 10.0f, float settle = 1.0f)
{
    GravityFilter f(config(leverArm));
    f.reset(upFrom(0.0f, 0.0f));
    float peak = 0.0f;
    const int n = static_cast<int>(duration / kDt);
    for (int i = 0; i < n; ++i) {
        const float t = i * kDt;
        f.update(m.input(t));
        float th, thd, thdd;
        m.at(t + kDt, th, thd, thdd);  // the filter has propagated to t + dt
        if (t > settle) {
            const float e = degErr(f.theta(), th);
            if (e > peak) peak = e;
        }
    }
    return peak;
}

}  // namespace

// Static poses at 0, +-10, +-90, +-179 and 180 deg, side lean 0 and 20 deg,
// starting 5 deg off: theta error < 0.05 deg after settling (30 s: the bias
// integrator takes up part of the start error and sheds it with tau ~ 9 s).
static void test_static_poses()
{
    const float thetas[] = {0.0f, 10.0f, -10.0f, 90.0f, -90.0f, 179.0f, -179.0f, 180.0f};
    const float lambdas[] = {0.0f, 20.0f};
    for (float lamDeg : lambdas) {
        for (float thDeg : thetas) {
            const float th = thDeg * kDeg, lam = lamDeg * kDeg;
            const Vec3 u   = upFrom(th, lam);
            GravityFilter f(config());
            f.reset(upFrom(th + 5.0f * kDeg, lam));
            for (int i = 0; i < 30 * 960; ++i) f.update(staticInput(u));
            char msg[64];
            snprintf(msg, sizeof msg, "theta %.0f, lean %.0f", thDeg, lamDeg);
            TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.05f, degErr(f.theta(), th), msg);
            TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.05f, degErr(f.lambda(), lam), msg);
            TEST_ASSERT_TRUE_MESSAGE(f.poseValid(), msg);
        }
    }
}

// The first sample sets u straight from the accelerometer.
static void test_first_sample_initialises()
{
    GravityFilter f(config());
    f.reset();
    const float th = 135.0f * kDeg;
    f.update(staticInput(upFrom(th, 0.0f)));
    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, degErr(f.theta(), th));
}

// Constant rotation about y through full turns (inverted included):
// tracking error < 0.1 deg.
static void test_constant_rotation()
{
    const float rate = 90.0f * kDeg;  // rad/s
    GravityFilter f(config());
    f.reset(upFrom(0.0f, 0.0f));
    float peak = 0.0f;
    for (int i = 0; i < 10 * 960; ++i) {
        const float t = i * kDt;
        f.update(rotatingInput(upFrom(rate * t, 0.0f), Vec3(0.0f, rate, 0.0f)));
        if (t > 0.5f) {
            const float e = degErr(f.theta(), rate * (t + kDt));
            if (e > peak) peak = e;
        }
    }
    TEST_ASSERT_LESS_THAN_FLOAT(0.1f, peak);
}

// Gyro bias 0.02 rad/s on y: converges, theta error < 0.2 deg after 30 s.
static void test_gyro_bias()
{
    GravityFilter f(config());
    const Vec3 u = upFrom(0.0f, 0.0f);
    f.reset(u);
    for (int i = 0; i < 30 * 960; ++i) f.update(staticInput(u, Vec3(0.0f, 0.02f, 0.0f)));
    TEST_ASSERT_LESS_THAN_FLOAT(0.2f, degErr(f.theta(), 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.02f, f.bias().y);
}

// (a) Wheel mode, body swinging about the axle: theta = 10 deg sin(2 pi 1.5 t).
static void test_lever_arm_wheel_swing()
{
    const PitchMotion m{10.0f * kDeg, 1.5f, ContactMode::Wheel};
    const float on  = peakError(m, true);
    const float off = peakError(m, false);
    char msg[96];
    snprintf(msg, sizeof msg, "wheel swing: peak error %.3f deg with compensation, %.3f deg without",
             on, off);
    TEST_MESSAGE(msg);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, on);
}

// (b) Ring mode, rolling on the rings: theta = 40 deg sin(2 pi 0.8 t).
static void test_lever_arm_ring_roll()
{
    const PitchMotion m{40.0f * kDeg, 0.8f, ContactMode::Ring};
    const float on  = peakError(m, true);
    const float off = peakError(m, false);
    char msg[96];
    snprintf(msg, sizeof msg, "ring roll: peak error %.3f deg with compensation, %.3f deg without",
             on, off);
    TEST_MESSAGE(msg);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, on);
}

// theta crossing +-180 deg gives no jump in wrap(theta - theta_ref).
static void test_wrap_through_180()
{
    const float thetaRef = kPi;
    const float rate     = 60.0f * kDeg;
    GravityFilter f(config());
    f.reset(upFrom(150.0f * kDeg, 0.0f));
    float prev   = 0.0f;
    bool first   = true;
    float maxJmp = 0.0f;
    for (int i = 0; i < 2 * 960; ++i) {  // 150 -> 270 deg
        const float th = 150.0f * kDeg + rate * i * kDt;
        f.update(rotatingInput(upFrom(th, 0.0f), Vec3(0.0f, rate, 0.0f)));
        const float e = wrapPi(f.theta() - thetaRef);
        if (!first && fabsf(e - prev) > maxJmp) maxJmp = fabsf(e - prev);
        prev  = e;
        first = false;
    }
    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, maxJmp);  // one step is 0.0011 rad
}

static void test_wrap_function()
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kPi, wrapPi(kPi));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kPi, wrapPi(-kPi));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -kPi + 0.1f, wrapPi(kPi + 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, wrapPi(0.5f + 4.0f * kPi));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, wrapPi(-0.5f - 6.0f * kPi));
}

// Axle near vertical: pose invalid.
static void test_pose_valid()
{
    GravityFilter f(config());
    f.reset(upFrom(0.0f, 59.0f * kDeg));
    TEST_ASSERT_TRUE(f.poseValid());
    f.reset(upFrom(0.0f, 61.0f * kDeg));
    TEST_ASSERT_FALSE(f.poseValid());
}

// Turn rate is about world vertical: right side up and inverted.
static void test_turn_rate_and_heading()
{
    const float poses[] = {0.0f, 180.0f};
    for (float thDeg : poses) {
        const Vec3 u = upFrom(thDeg * kDeg, 0.0f);
        GravityFilter f(config());
        f.reset(u);
        // Turning left in the world at 1 rad/s: body rate is +1 rad/s about world up.
        const Vec3 gyro = u * 1.0f;
        for (int i = 0; i < 960; ++i) f.update(rotatingInput(u, gyro));
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, f.psiDot());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, f.psi());
    }
}

// Changing the mounting offset: state re-expressed, no jump.
static void test_rotate_about_y()
{
    GravityFilter f(config());
    const float th = 5.0f * kDeg;
    f.reset(upFrom(th, 0.0f));
    f.rotateAboutY(th);  // inputs rotated by the same from now on
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, degErr(f.theta(), 0.0f));
    for (int i = 0; i < 960; ++i) f.update(staticInput(rotateY(upFrom(th, 0.0f), th)));
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, degErr(f.theta(), 0.0f));
}

// h, h' and h'' against a circle and against numerical derivatives.
static void test_ring_support()
{
    const RingSupport circle = ringSupport(0.7f, 0.07f, 0.07f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.07f, circle.h);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, circle.dh);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, circle.d2h);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kRingB, ringSupport(0.0f, kRingA, kRingB).h);        // upright
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kRingA, ringSupport(kPi / 2, kRingA, kRingB).h);     // on its front
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kRingB, ringSupport(kPi, kRingA, kRingB).h);         // inverted

    // Full ellipse: analytic derivatives against numerical ones.
    auto hEll = [](double th) {
        const double s = sin(th), c = cos(th);
        return sqrt(double(kRingA) * kRingA * s * s + double(kRingB) * kRingB * c * c);
    };
    const double e = 1e-4;
    for (float thDeg = -170.0f; thDeg <= 170.0f; thDeg += 17.0f) {
        const double th = thDeg * kDeg;
        const RingSupport s = ringSupport(static_cast<float>(th), kRingA, kRingB);
        const double dh  = (hEll(th + e) - hEll(th - e)) / (2 * e);
        const double d2h = (hEll(th + e) - 2 * hEll(th) + hEll(th - e)) / (e * e);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, static_cast<float>(hEll(th)), s.h);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, static_cast<float>(dh), s.dh);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, static_cast<float>(d2h), s.d2h);
    }
}

// The cut ring: support against brute force over the remaining ring points,
// all the way round, plus h' numerically (continuous across the corners).
static void test_cut_ring_support()
{
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 21.6f, kRing.cutHalfAngle() / kDeg);  // hand calculation
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02485f, kRing.cornerX());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.06974f, kRing.cornerZ());

    for (float thDeg = -180.0f; thDeg <= 180.0f; thDeg += 2.5f) {
        const double th = thDeg * kDeg;
        const RingSupport s = kRing.support(static_cast<float>(th));
        char msg[48];
        snprintf(msg, sizeof msg, "theta %.1f deg", thDeg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2e-6f, static_cast<float>(ringHeightBrute(th)), s.h, msg);
        // At theta = 0 the ring stands on both corners: h has a kink there
        // (slope -x to +x). Ring mode never gets near it (|theta| > ~20 deg).
        if (thDeg == 0.0f) continue;
        const double e  = 1e-3;  // ringHeight is float: a smaller step is rounding noise
        const double dh = (ringHeight(th + e) - ringHeight(th - e)) / (2 * e);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2e-5f, static_cast<float>(dh), s.dh, msg);
    }

    // A cut that misses the ring changes nothing.
    const RingShape uncut(kRingA, kRingB, kDc, 0.020f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, uncut.cutHalfAngle());
    TEST_ASSERT_EQUAL_FLOAT(ringSupport(0.3f, kRingA, kRingB).h, uncut.support(0.3f).h);
}

// Wheels-touch limit: matches acos((R - r_w)/d_c) for an uncut circle; for
// the uncut ellipse about 19.7 deg; with the 66 mm cut about 20.3 deg.
static void test_wheel_contact_limit()
{
    const RingShape circle(0.15f, 0.15f, 0.12f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, acosf((0.15f - 0.06f) / 0.12f),
                             wheelContactLimit(circle, 0.12f, 0.06f));

    const RingShape uncut(kRingA, kRingB, kDc, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 19.7f, wheelContactLimit(uncut, kDc, kRw) / kDeg);

    const float thC = wheelContactLimit(kRing, kDc, kRw);
    char msg[64];
    snprintf(msg, sizeof msg, "robot: wheels touch for |theta| < %.2f deg", thC / kDeg);
    TEST_MESSAGE(msg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kRing.support(thC).h, kDc * cosf(thC) + kRw);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.3f, thC / kDeg);

    const RingShape small(kRingA, kRingB, kDc, kCutR);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, wheelContactLimit(small, kDc, 0.020f));  // never touch
}

static void test_contact_mode_hysteresis()
{
    const float thetaC = 40.0f * kDeg, hyst = 2.0f * kDeg;
    ContactMode m = ContactMode::Wheel;
    m = nextContactMode(m, 41.0f * kDeg, thetaC, hyst);
    TEST_ASSERT_TRUE(m == ContactMode::Wheel);
    m = nextContactMode(m, -42.5f * kDeg, thetaC, hyst);
    TEST_ASSERT_TRUE(m == ContactMode::Ring);
    m = nextContactMode(m, 39.0f * kDeg, thetaC, hyst);
    TEST_ASSERT_TRUE(m == ContactMode::Ring);
    m = nextContactMode(m, 37.5f * kDeg, thetaC, hyst);
    TEST_ASSERT_TRUE(m == ContactMode::Wheel);
    m = nextContactMode(m, 180.0f * kDeg, thetaC, hyst);
    TEST_ASSERT_TRUE(m == ContactMode::Ring);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_static_poses);
    RUN_TEST(test_first_sample_initialises);
    RUN_TEST(test_constant_rotation);
    RUN_TEST(test_gyro_bias);
    RUN_TEST(test_lever_arm_wheel_swing);
    RUN_TEST(test_lever_arm_ring_roll);
    RUN_TEST(test_wrap_through_180);
    RUN_TEST(test_wrap_function);
    RUN_TEST(test_pose_valid);
    RUN_TEST(test_turn_rate_and_heading);
    RUN_TEST(test_rotate_about_y);
    RUN_TEST(test_ring_support);
    RUN_TEST(test_cut_ring_support);
    RUN_TEST(test_wheel_contact_limit);
    RUN_TEST(test_contact_mode_hysteresis);
    return UNITY_END();
}
