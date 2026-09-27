#include <gtest/gtest.h>
#include "bno08x_driver/bno08x.hpp"

TEST(SampleAgeTest, SimpleDifference) {
    EXPECT_EQ(BNO08x::sample_age_us(1500, 1000), 500u);
}

TEST(SampleAgeTest, HandlesHostCounterWrap) {
    // Sample taken just before the 32-bit host counter wrapped, read just after
    EXPECT_EQ(BNO08x::sample_age_us(100, 0xFFFFFF00ull), 356u);
}

TEST(SampleAgeTest, IgnoresRolloverBitsOfSampleTime) {
    // The sh2 library extends sample times to 64 bits by counting rollovers
    const uint64_t sample = (3ull << 32) + 2000;
    EXPECT_EQ(BNO08x::sample_age_us(2600, sample), 600u);
}

TEST(SampleAgeTest, FutureSampleIsLarge) {
    // A sample "newer" than now wraps to a huge age, which the driver rejects
    EXPECT_GT(BNO08x::sample_age_us(1000, 1500), 1000000u);
}
