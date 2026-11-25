// Seqlock: one writer (the control task), any number of readers on other
// tasks (spec §8.2 rule 3). The writer never blocks; a reader retries if the
// writer was mid-update.
#pragma once

#include <atomic>
#include <stdint.h>

template <typename T>
class Snapshot {
public:
    void write(const T &value)
    {
        const uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);  // odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);
        data_ = value;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_relaxed);
    }

    // Returns false only if it couldn't get a consistent copy in maxTries.
    bool read(T &out, int maxTries = 1000) const
    {
        for (int i = 0; i < maxTries; ++i) {
            const uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u) continue;
            out = data_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s1) return true;
        }
        return false;
    }

    // Number of writes so far.
    uint32_t version() const { return seq_.load(std::memory_order_acquire) / 2; }

private:
    std::atomic<uint32_t> seq_{0};
    T data_{};
};
