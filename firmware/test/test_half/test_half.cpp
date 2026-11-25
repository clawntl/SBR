// Host unit test for halfToFloat (spec §5.2).
#include <math.h>
#include <unity.h>

#include "util/Half.h"

void setUp() {}
void tearDown() {}

static void test_exact_values()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, halfToFloat(0x0000));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, halfToFloat(0x3C00));
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, halfToFloat(0xC000));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, halfToFloat(0x3800));
    TEST_ASSERT_EQUAL_FLOAT(65504.0f, halfToFloat(0x7BFF));          // largest normal
    TEST_ASSERT_EQUAL_FLOAT(0.333251953125f, halfToFloat(0x3555));   // nearest to 1/3
    TEST_ASSERT_EQUAL_FLOAT(0.99951171875f, halfToFloat(0x3BFF));    // just below 1
}

static void test_negative_zero()
{
    const float f = halfToFloat(0x8000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f);
    TEST_ASSERT_TRUE(signbit(f));
}

static void test_subnormals()
{
    TEST_ASSERT_EQUAL_FLOAT(ldexpf(1.0f, -24), halfToFloat(0x0001));             // smallest
    TEST_ASSERT_EQUAL_FLOAT(ldexpf(1023.0f, -24), halfToFloat(0x03FF));          // largest
    TEST_ASSERT_EQUAL_FLOAT(-ldexpf(512.0f, -24), halfToFloat(0x8200));
}

static void test_inf_and_nan()
{
    TEST_ASSERT_TRUE(isinf(halfToFloat(0x7C00)) && halfToFloat(0x7C00) > 0);
    TEST_ASSERT_TRUE(isinf(halfToFloat(0xFC00)) && halfToFloat(0xFC00) < 0);
    TEST_ASSERT_TRUE(isnan(halfToFloat(0x7E00)));
}

// Every finite half round-trips through the exact formula.
static void test_all_finite_match_formula()
{
    for (uint32_t h = 0; h < 0x10000; ++h) {
        const uint32_t exp  = (h >> 10) & 0x1F;
        const uint32_t mant = h & 0x3FF;
        if (exp == 0x1F) continue;
        const float mag = exp == 0 ? ldexpf(static_cast<float>(mant), -24)
                                   : ldexpf(1.0f + mant / 1024.0f, static_cast<int>(exp) - 15);
        const float expected = (h & 0x8000) ? -mag : mag;
        TEST_ASSERT_EQUAL_FLOAT(expected, halfToFloat(static_cast<uint16_t>(h)));
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_exact_values);
    RUN_TEST(test_negative_zero);
    RUN_TEST(test_subnormals);
    RUN_TEST(test_inf_and_nan);
    RUN_TEST(test_all_finite_match_formula);
    return UNITY_END();
}
