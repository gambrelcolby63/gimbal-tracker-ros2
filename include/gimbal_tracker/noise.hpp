#pragma once

#include <cmath>
#include <random>

namespace gimbal_tracker {

// Seeded scalar Gaussian sampler. stddev <= 0 yields exact zeros.
class GaussianNoise {
 public:
  explicit GaussianNoise(unsigned seed = 1) : rng_(seed) {}

  void reseed(unsigned seed) { rng_.seed(seed); }

  double sample(double stddev) {
    if (!(stddev > 0.0) || !std::isfinite(stddev)) {
      return 0.0;
    }
    return stddev * dist_(rng_);
  }

 private:
  std::mt19937 rng_;
  std::normal_distribution<double> dist_{0.0, 1.0};
};

}  // namespace gimbal_tracker
