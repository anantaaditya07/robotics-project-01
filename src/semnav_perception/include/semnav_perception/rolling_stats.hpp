// Rolling window percentiles for stage timing (architecture 7.1 "Metrics"). ROS-free.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

namespace semnav_perception {

/// Keeps the last `capacity` samples and reports nearest-rank percentiles.
class RollingStats {
 public:
  explicit RollingStats(std::size_t capacity) : capacity_(capacity) {
    if (capacity == 0) {
      throw std::invalid_argument("RollingStats: capacity must be > 0");
    }
  }

  void add(double v) {
    samples_.push_back(v);
    if (samples_.size() > capacity_) {
      samples_.pop_front();
    }
  }

  std::size_t size() const { return samples_.size(); }

  /// Nearest-rank percentile, p in [0, 100]: the smallest sample with at least p% of the
  /// samples <= it. Returns NaN when empty.
  double percentile(double p) const {
    if (p < 0.0 || p > 100.0) {
      throw std::invalid_argument("RollingStats: percentile must be in [0, 100]");
    }
    if (samples_.empty()) {
      return std::nan("");
    }
    std::vector<double> v(samples_.begin(), samples_.end());
    const auto n = v.size();
    const auto rank = static_cast<std::size_t>(std::ceil(p / 100.0 * static_cast<double>(n)));
    const std::size_t idx = rank == 0 ? 0 : rank - 1;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(idx), v.end());
    return v[idx];
  }

 private:
  std::size_t capacity_;
  std::deque<double> samples_;
};

}  // namespace semnav_perception
