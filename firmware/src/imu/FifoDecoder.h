// LSM6DSV16X FIFO word decoder (spec §5.2). Pure logic, unit-tested on the
// host.
//
// A FIFO word is 7 bytes: a tag byte (sensor tag in bits 7:3, 2-bit time-slot
// counter in bits 2:1) and 6 data bytes, little-endian. Gyro and accel words
// from the same time slot carry the same counter and are paired into one
// sample. A pair can be split across two drains; the half that arrived first
// waits here.
#pragma once

#include <stdint.h>

namespace FifoTag {
constexpr uint8_t kGyro      = 0x01;
constexpr uint8_t kAccel     = 0x02;
constexpr uint8_t kTimestamp = 0x04;
constexpr uint8_t kCfgChange = 0x05;
constexpr uint8_t kSflpGame  = 0x13;
constexpr uint8_t kSflpGbias = 0x16;
constexpr uint8_t kSflpGrav  = 0x17;
}  // namespace FifoTag

constexpr int kFifoWordBytes = 7;

struct ImuPair {
    int16_t gyro[3];   // raw, sensor axes
    int16_t accel[3];  // raw, sensor axes
    uint32_t ts;       // IMU timestamp of the latest timestamp word (LSB ~21.7 us)
};

// Sink interface (duck-typed):
//   void onPair(const ImuPair &);
//   void onGravity(const int16_t v[3]);   // SFLP gravity, raw (0.061 mg/LSB)
//   void onGame(const uint16_t h[3]);     // SFLP game rotation x, y, z, half-floats
//   void onBadTag(uint8_t tag);
//   void onUnpaired();                    // a gyro or accel word without its partner
class FifoDecoder {
public:
    template <typename Sink>
    void feed(const uint8_t *word, Sink &sink)
    {
        const uint8_t tag = word[0] >> 3;
        const uint8_t cnt = (word[0] >> 1) & 0x3;
        const uint8_t *d  = word + 1;

        switch (tag) {
            case FifoTag::kGyro:
                take(gyro_, cnt, d, sink);
                break;
            case FifoTag::kAccel:
                take(accel_, cnt, d, sink);
                break;
            case FifoTag::kTimestamp:
                ts_ = static_cast<uint32_t>(d[0]) | static_cast<uint32_t>(d[1]) << 8 |
                      static_cast<uint32_t>(d[2]) << 16 | static_cast<uint32_t>(d[3]) << 24;
                break;
            case FifoTag::kSflpGrav: {
                int16_t v[3];
                for (int i = 0; i < 3; ++i) v[i] = s16(d + 2 * i);
                sink.onGravity(v);
                break;
            }
            case FifoTag::kSflpGame: {
                uint16_t h[3];
                for (int i = 0; i < 3; ++i) h[i] = u16(d + 2 * i);
                sink.onGame(h);
                break;
            }
            case FifoTag::kCfgChange:
            case FifoTag::kSflpGbias:
                break;
            default:
                sink.onBadTag(tag);
                break;
        }

        if (gyro_.valid && accel_.valid) {
            if (gyro_.cnt == accel_.cnt) {
                ImuPair p;
                for (int i = 0; i < 3; ++i) {
                    p.gyro[i]  = gyro_.v[i];
                    p.accel[i] = accel_.v[i];
                }
                p.ts = ts_;
                gyro_.valid = accel_.valid = false;
                sink.onPair(p);
            } else {
                // Different time slots: the one that didn't just arrive is
                // stale.
                Half &stale = (tag == FifoTag::kGyro) ? accel_ : gyro_;
                stale.valid = false;
                sink.onUnpaired();
            }
        }
    }

    void reset() { gyro_.valid = accel_.valid = false; }

private:
    struct Half {
        bool valid = false;
        uint8_t cnt = 0;
        int16_t v[3] = {0, 0, 0};
    };

    static int16_t s16(const uint8_t *p)
    {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (p[1] << 8)));
    }
    static uint16_t u16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

    template <typename Sink>
    static void take(Half &h, uint8_t cnt, const uint8_t *d, Sink &sink)
    {
        if (h.valid) sink.onUnpaired();  // previous one never got its partner
        h.valid = true;
        h.cnt   = cnt;
        for (int i = 0; i < 3; ++i) h.v[i] = s16(d + 2 * i);
    }

    Half gyro_;
    Half accel_;
    uint32_t ts_ = 0;
};
