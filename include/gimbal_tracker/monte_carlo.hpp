#pragma once

#include "gimbal_tracker/kalman.hpp"

#include <vector>

namespace gimbal_tracker {

// Matched-noise consistency experiment. The truth initial state is drawn from
// the filter prior, and the truth process noise is the same piecewise-constant
// acceleration or jerk the filter's Q assumes. A consistent filter then has
// mean NEES near the state dimension and mean NIS near the measurement dimension.
struct MonteCarloConfig {
  TargetKalmanFilter::Model model{TargetKalmanFilter::Model::ConstantVelocity};
  int runs{50};
  double dt{0.05};
  double duration{5.0};
  double accel_std{1.0};
  double jerk_std{1.0};
  double meas_std{0.5};
  double pos_var{4.0};
  double vel_var{4.0};
  double acc_var{4.0};
  unsigned seed{7};
  bool bearing_range{false};
  double azimuth_std{0.002};
  double elevation_std{0.002};
  double range_std{0.4};
  Vec3 prior_position{0.0, 0.0, 0.0};
  Vec3 prior_velocity{0.0, 0.0, 0.0};
  Vec3 prior_acceleration{0.0, 0.0, 0.0};
  Vec3 muzzle{0.0, 0.0, 0.0};
};

struct MonteCarloResult {
  int runs{0};
  int state_dim{0};
  double dt{0.0};
  double average_nees{0.0};
  double average_nis{0.0};
  std::vector<double> time;
  std::vector<double> mean_nees;
  std::vector<double> mean_nis;
  std::vector<double> pos_rmse;
  std::vector<double> vel_rmse;
};

MonteCarloResult runConsistencyMonteCarlo(const MonteCarloConfig& config);

}  // namespace gimbal_tracker
