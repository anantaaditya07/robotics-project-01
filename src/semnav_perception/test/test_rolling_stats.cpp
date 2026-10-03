#include <gtest/gtest.h>

#include <cmath>
#include <semnav_perception/rolling_stats.hpp>

using semnav_perception::RollingStats;

TEST(RollingStats, NearestRankOnOneToHundred) {
  RollingStats s(100);
  for (int i = 1; i <= 100; ++i) {
    s.add(i);
  }
  // Nearest rank: p50 -> rank 50 -> 50; p95 -> rank 95 -> 95.
  EXPECT_DOUBLE_EQ(s.percentile(50), 50.0);
  EXPECT_DOUBLE_EQ(s.percentile(95), 95.0);
  EXPECT_DOUBLE_EQ(s.percentile(100), 100.0);
  EXPECT_DOUBLE_EQ(s.percentile(0), 1.0);
}

TEST(RollingStats, WindowDropsOldestSamples) {
  RollingStats s(3);
  for (double v : {100.0, 1.0, 2.0, 3.0}) {
    s.add(v);
  }
  EXPECT_EQ(s.size(), 3U);
  EXPECT_DOUBLE_EQ(s.percentile(100), 3.0);  // 100 has left the window
}

TEST(RollingStats, EmptyIsNanAndBadArgsThrow) {
  RollingStats s(4);
  EXPECT_TRUE(std::isnan(s.percentile(50)));
  EXPECT_THROW(s.percentile(101), std::invalid_argument);
  EXPECT_THROW(RollingStats(0), std::invalid_argument);
}
