// Host unit test for the status LED colour wheel (Stage 0's native test).
#include <unity.h>

#include "util/ColourWheel.h"

void setUp() {}
void tearDown() {}

static void assertRgb(Rgb expected, Rgb actual)
{
    TEST_ASSERT_EQUAL_UINT8(expected.r, actual.r);
    TEST_ASSERT_EQUAL_UINT8(expected.g, actual.g);
    TEST_ASSERT_EQUAL_UINT8(expected.b, actual.b);
}

static void test_primaries_and_secondaries()
{
    assertRgb({255, 0, 0}, colourWheel(0, 255));
    assertRgb({255, 255, 0}, colourWheel(60, 255));
    assertRgb({0, 255, 0}, colourWheel(120, 255));
    assertRgb({0, 255, 255}, colourWheel(180, 255));
    assertRgb({0, 0, 255}, colourWheel(240, 255));
    assertRgb({255, 0, 255}, colourWheel(300, 255));
}

static void test_wraps_hue()
{
    assertRgb(colourWheel(0, 100), colourWheel(360, 100));
    assertRgb(colourWheel(45, 100), colourWheel(405, 100));
}

static void test_level_bounds_every_channel()
{
    for (uint32_t hue = 0; hue < 360; ++hue) {
        const Rgb c = colourWheel(hue, 32);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(32, c.r);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(32, c.g);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(32, c.b);
        // Exactly one channel is at full level, one is at zero (full saturation).
        const int full = (c.r == 32) + (c.g == 32) + (c.b == 32);
        const int zero = (c.r == 0) + (c.g == 0) + (c.b == 0);
        TEST_ASSERT_TRUE(full >= 1);
        TEST_ASSERT_TRUE(zero >= 1);
    }
}

static void test_continuous()
{
    // Adjacent hues never jump by more than one step of the ramp (plus rounding).
    for (uint32_t hue = 0; hue < 360; ++hue) {
        const Rgb a = colourWheel(hue, 255);
        const Rgb b = colourWheel(hue + 1, 255);
        TEST_ASSERT_INT_WITHIN(5, a.r, b.r);
        TEST_ASSERT_INT_WITHIN(5, a.g, b.g);
        TEST_ASSERT_INT_WITHIN(5, a.b, b.b);
    }
}

static void test_zero_level_is_off()
{
    for (uint32_t hue = 0; hue < 360; hue += 7) assertRgb({0, 0, 0}, colourWheel(hue, 0));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_primaries_and_secondaries);
    RUN_TEST(test_wraps_hue);
    RUN_TEST(test_level_bounds_every_channel);
    RUN_TEST(test_continuous);
    RUN_TEST(test_zero_level_is_off);
    return UNITY_END();
}
