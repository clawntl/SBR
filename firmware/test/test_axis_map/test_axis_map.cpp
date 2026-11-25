// Host unit test for the sensor-to-body axis map (spec §5.3).
#include <math.h>
#include <unity.h>

#include "util/AxisMap.h"

void setUp() {}
void tearDown() {}

static constexpr AxisMatrix kIdentity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
// Body x = sensor +Y, body y = sensor -X, body z = sensor +Z (90 deg about z).
static constexpr AxisMatrix kRotZ = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}};
static_assert(isProperAxisMap(kIdentity), "identity");
static_assert(isProperAxisMap(kRotZ), "rot z");

static void test_validity()
{
    const AxisMatrix mirror  = {{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}};   // det -1
    const AxisMatrix twoX    = {{1, 0, 0}, {1, 0, 0}, {0, 0, 1}};    // repeated axis
    const AxisMatrix scaled  = {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const AxisMatrix upside  = {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}};  // 180 deg about x
    TEST_ASSERT_FALSE(isProperAxisMap(mirror));
    TEST_ASSERT_FALSE(isProperAxisMap(twoX));
    TEST_ASSERT_FALSE(isProperAxisMap(scaled));
    TEST_ASSERT_TRUE(isProperAxisMap(upside));
}

static void test_vector()
{
    const float in[3] = {1.0f, 2.0f, 3.0f};
    float out[3];
    applyAxisMap(kRotZ, in, out);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, out[0]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out[2]);
}

// Quaternion remap equals the similarity transform M R M^T, checked by
// rotating a vector both ways.
static void rotate(const float q[4], const float v[3], float out[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float R[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
        {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
        {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)},
    };
    for (int i = 0; i < 3; ++i) out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
}

static void test_quaternion_is_similarity_transform()
{
    // 30 deg about sensor axis (1, 2, 3)/|.|
    const float n = sqrtf(14.0f), h = 0.2617994f;
    const float qs[4] = {cosf(h), sinf(h) / n, 2 * sinf(h) / n, 3 * sinf(h) / n};
    float qb[4];
    applyAxisMapQuat(kRotZ, qs, qb);
    TEST_ASSERT_EQUAL_FLOAT(qs[0], qb[0]);

    // For any body vector vb: R_b vb == M R_s M^T vb.
    const float vb[3] = {0.3f, -0.7f, 0.4f};
    float viaBody[3];
    rotate(qb, vb, viaBody);

    float vs[3], rs[3], viaSensor[3];
    const float MT[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};  // transpose of kRotZ
    for (int i = 0; i < 3; ++i) vs[i] = MT[i][0] * vb[0] + MT[i][1] * vb[1] + MT[i][2] * vb[2];
    rotate(qs, vs, rs);
    applyAxisMap(kRotZ, rs, viaSensor);

    for (int i = 0; i < 3; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-6f, viaSensor[i], viaBody[i]);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_validity);
    RUN_TEST(test_vector);
    RUN_TEST(test_quaternion_is_similarity_transform);
    return UNITY_END();
}
