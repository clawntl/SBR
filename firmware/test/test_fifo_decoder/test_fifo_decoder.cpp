// Host unit test for the LSM6DSV16X FIFO word decoder (spec §5.2).
#include <string.h>
#include <unity.h>

#include "imu/FifoDecoder.h"

void setUp() {}
void tearDown() {}

namespace {

struct Recorder {
    int pairs = 0, gravity = 0, game = 0, badTags = 0, unpaired = 0;
    ImuPair lastPair{};
    int16_t lastGravity[3]{};
    uint16_t lastGame[3]{};
    uint8_t lastBadTag = 0;

    void onPair(const ImuPair &p) { ++pairs; lastPair = p; }
    void onGravity(const int16_t v[3]) { ++gravity; memcpy(lastGravity, v, sizeof lastGravity); }
    void onGame(const uint16_t h[3]) { ++game; memcpy(lastGame, h, sizeof lastGame); }
    void onBadTag(uint8_t t) { ++badTags; lastBadTag = t; }
    void onUnpaired() { ++unpaired; }
};

struct Word {
    uint8_t b[kFifoWordBytes];
};

Word word(uint8_t tag, uint8_t cnt, int16_t x, int16_t y, int16_t z)
{
    Word w;
    w.b[0] = static_cast<uint8_t>(tag << 3 | (cnt & 3) << 1);
    const int16_t v[3] = {x, y, z};
    for (int i = 0; i < 3; ++i) {
        w.b[1 + 2 * i] = static_cast<uint8_t>(v[i] & 0xFF);
        w.b[2 + 2 * i] = static_cast<uint8_t>((static_cast<uint16_t>(v[i]) >> 8) & 0xFF);
    }
    return w;
}

Word timestamp(uint8_t cnt, uint32_t ts)
{
    Word w = {};
    w.b[0] = static_cast<uint8_t>(FifoTag::kTimestamp << 3 | (cnt & 3) << 1);
    for (int i = 0; i < 4; ++i) w.b[1 + i] = static_cast<uint8_t>(ts >> (8 * i));
    return w;
}

}  // namespace

static void test_one_cycle_pairs()
{
    FifoDecoder d;
    Recorder r;
    const Word ws[] = {timestamp(1, 0x12345678), word(FifoTag::kGyro, 1, 1, -2, 3),
                       word(FifoTag::kAccel, 1, 4096, -5, -32768)};
    for (const Word &w : ws) d.feed(w.b, r);

    TEST_ASSERT_EQUAL(1, r.pairs);
    TEST_ASSERT_EQUAL(0, r.unpaired);
    TEST_ASSERT_EQUAL_INT16(1, r.lastPair.gyro[0]);
    TEST_ASSERT_EQUAL_INT16(-2, r.lastPair.gyro[1]);
    TEST_ASSERT_EQUAL_INT16(3, r.lastPair.gyro[2]);
    TEST_ASSERT_EQUAL_INT16(4096, r.lastPair.accel[0]);
    TEST_ASSERT_EQUAL_INT16(-5, r.lastPair.accel[1]);
    TEST_ASSERT_EQUAL_INT16(-32768, r.lastPair.accel[2]);
    TEST_ASSERT_EQUAL_UINT32(0x12345678u, r.lastPair.ts);
}

static void test_accel_first_also_pairs()
{
    FifoDecoder d;
    Recorder r;
    const Word a = word(FifoTag::kAccel, 2, 7, 8, 9);
    const Word g = word(FifoTag::kGyro, 2, 1, 2, 3);
    d.feed(a.b, r);
    d.feed(g.b, r);
    TEST_ASSERT_EQUAL(1, r.pairs);
    TEST_ASSERT_EQUAL_INT16(7, r.lastPair.accel[0]);
    TEST_ASSERT_EQUAL_INT16(3, r.lastPair.gyro[2]);
}

static void test_pair_split_across_drains()
{
    FifoDecoder d;
    Recorder r;
    const Word g = word(FifoTag::kGyro, 3, 1, 1, 1);
    d.feed(g.b, r);  // first drain ends here
    TEST_ASSERT_EQUAL(0, r.pairs);
    const Word a = word(FifoTag::kAccel, 3, 2, 2, 2);
    d.feed(a.b, r);  // second drain
    TEST_ASSERT_EQUAL(1, r.pairs);
    TEST_ASSERT_EQUAL(0, r.unpaired);
}

static void test_mismatched_slots_drop_stale_half()
{
    FifoDecoder d;
    Recorder r;
    const Word g0 = word(FifoTag::kGyro, 0, 10, 0, 0);
    const Word a1 = word(FifoTag::kAccel, 1, 20, 0, 0);
    const Word g1 = word(FifoTag::kGyro, 1, 11, 0, 0);
    d.feed(g0.b, r);
    d.feed(a1.b, r);  // gyro slot 0 is stale: dropped
    TEST_ASSERT_EQUAL(0, r.pairs);
    TEST_ASSERT_EQUAL(1, r.unpaired);
    d.feed(g1.b, r);
    TEST_ASSERT_EQUAL(1, r.pairs);
    TEST_ASSERT_EQUAL_INT16(11, r.lastPair.gyro[0]);
    TEST_ASSERT_EQUAL_INT16(20, r.lastPair.accel[0]);
}

static void test_repeated_gyro_counts_unpaired()
{
    FifoDecoder d;
    Recorder r;
    const Word g0 = word(FifoTag::kGyro, 0, 1, 0, 0);
    const Word g1 = word(FifoTag::kGyro, 1, 2, 0, 0);
    const Word a1 = word(FifoTag::kAccel, 1, 3, 0, 0);
    d.feed(g0.b, r);
    d.feed(g1.b, r);
    d.feed(a1.b, r);
    TEST_ASSERT_EQUAL(1, r.unpaired);
    TEST_ASSERT_EQUAL(1, r.pairs);
    TEST_ASSERT_EQUAL_INT16(2, r.lastPair.gyro[0]);
}

static void test_sflp_words()
{
    FifoDecoder d;
    Recorder r;
    const Word grav = word(FifoTag::kSflpGrav, 0, 0, -100, 16393);
    const Word game = word(FifoTag::kSflpGame, 0, 0x3800, static_cast<int16_t>(0xB800), 0);
    d.feed(grav.b, r);
    d.feed(game.b, r);
    TEST_ASSERT_EQUAL(1, r.gravity);
    TEST_ASSERT_EQUAL_INT16(-100, r.lastGravity[1]);
    TEST_ASSERT_EQUAL_INT16(16393, r.lastGravity[2]);
    TEST_ASSERT_EQUAL(1, r.game);
    TEST_ASSERT_EQUAL_HEX16(0x3800, r.lastGame[0]);
    TEST_ASSERT_EQUAL_HEX16(0xB800, r.lastGame[1]);
    TEST_ASSERT_EQUAL(0, r.pairs);
}

static void test_bad_and_ignored_tags()
{
    FifoDecoder d;
    Recorder r;
    const Word cfg   = word(FifoTag::kCfgChange, 0, 0, 0, 0);
    const Word gbias = word(FifoTag::kSflpGbias, 0, 0, 0, 0);
    const Word empty = word(0x00, 0, 0, 0, 0);
    const Word temp  = word(0x03, 0, 0, 0, 0);
    d.feed(cfg.b, r);
    d.feed(gbias.b, r);
    TEST_ASSERT_EQUAL(0, r.badTags);
    d.feed(empty.b, r);
    d.feed(temp.b, r);
    TEST_ASSERT_EQUAL(2, r.badTags);
    TEST_ASSERT_EQUAL_UINT8(0x03, r.lastBadTag);
}

static void test_steady_stream()
{
    // 100 cycles: ts, gyro, accel, plus SFLP gravity every other cycle.
    FifoDecoder d;
    Recorder r;
    for (int i = 0; i < 100; ++i) {
        const uint8_t c = static_cast<uint8_t>(i & 3);
        const Word ws[] = {timestamp(c, 1000u + 48u * i), word(FifoTag::kGyro, c, i, 0, 0),
                           word(FifoTag::kAccel, c, -i, 0, 0)};
        for (const Word &w : ws) d.feed(w.b, r);
        if (i % 2) {
            const Word g = word(FifoTag::kSflpGrav, c, 0, 0, 16393);
            d.feed(g.b, r);
        }
        TEST_ASSERT_EQUAL_UINT32(1000u + 48u * i, r.lastPair.ts);
    }
    TEST_ASSERT_EQUAL(100, r.pairs);
    TEST_ASSERT_EQUAL(50, r.gravity);
    TEST_ASSERT_EQUAL(0, r.unpaired);
    TEST_ASSERT_EQUAL(0, r.badTags);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_one_cycle_pairs);
    RUN_TEST(test_accel_first_also_pairs);
    RUN_TEST(test_pair_split_across_drains);
    RUN_TEST(test_mismatched_slots_drop_stale_half);
    RUN_TEST(test_repeated_gyro_counts_unpaired);
    RUN_TEST(test_sflp_words);
    RUN_TEST(test_bad_and_ignored_tags);
    RUN_TEST(test_steady_stream);
    return UNITY_END();
}
