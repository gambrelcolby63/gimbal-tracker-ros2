#include "gimbal_tracker/monte_carlo.hpp"

#include "gimbal_tracker/lead.hpp"
#include "gimbal_tracker/noise.hpp"

#include <cmath>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

void samplePrior(TargetState& truth, const MonteCarloConfig& config, GaussianNoise& noise) {
  const double pos_std = std::sqrt(config.pos_var);
  const double vel_std = std::sqrt(config.vel_var);
  const double acc_std = std::sqrt(config.acc_var);
  truth = TargetState{};
  truth.position = config.prior_position;
  truth.velocity = config.prior_velocity;
  truth.acceleration = config.prior_acceleration;
  truth.position.x += noise.sample(pos_std);
  truth.position.y += noise.sample(pos_std);
  truth.position.z += noise.sample(pos_std);
  truth.velocity.x += noise.sample(vel_std);
  truth.velocity.y += noise.sample(vel_std);
  truth.velocity.z += noise.sample(vel_std);
  if (config.model == TargetKalmanFilter::Model::ConstantAcceleration) {
    truth.acceleration.x += noise.sample(acc_std);
    truth.acceleration.y += noise.sample(acc_std);
    truth.acceleration.z += noise.sample(acc_std);
  } else {
    truth.acceleration = {0.0, 0.0, 0.0};
  }
}

void propagateTruth(TargetState& truth, const MonteCarloConfig& config, GaussianNoise& noise) {
  const double dt = config.dt;
  if (config.model == TargetKalmanFilter::Model::ConstantVelocity) {
    const Vec3 accel{noise.sample(config.accel_std), noise.sample(config.accel_std),
                     noise.sample(config.accel_std)};
    truth.position = truth.position + truth.velocity * dt + accel * (0.5 * dt * dt);
    truth.velocity = truth.velocity + accel * dt;
    truth.acceleration = {0.0, 0.0, 0.0};
    return;
  }
  const Vec3 jerk{noise.sample(config.jerk_std), noise.sample(config.jerk_std),
                  noise.sample(config.jerk_std)};
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  truth.position = truth.position + truth.velocity * dt + truth.acceleration * (0.5 * dt2) +
                   jerk * (dt3 / 6.0);
  truth.velocity = truth.velocity + truth.acceleration * dt + jerk * (0.5 * dt2);
  truth.acceleration = truth.acceleration + jerk * dt;
}

void updateFilter(TargetKalmanFilter& filter, const TargetState& truth, const MonteCarloConfig& config,
                  GaussianNoise& noise) {
  if (!config.bearing_range) {
    Eigen::Vector3d measurement;
    measurement << truth.position.x + noise.sample(config.meas_std),
        truth.position.y + noise.sample(config.meas_std),
        truth.position.z + noise.sample(config.meas_std);
    const Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Identity() * (config.meas_std * config.meas_std);
    filter.updateCartesian(measurement, covariance);
    return;
  }

  const LookAngles look = lookAngles(config.muzzle, truth.position);
  Eigen::Vector3d measurement;
  measurement << wrapPi(look.azimuth + noise.sample(config.azimuth_std)),
      look.elevation + noise.sample(config.elevation_std),
      std::max(1.0e-3, look.range + noise.sample(config.range_std));
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = config.azimuth_std * config.azimuth_std;
  covariance(1, 1) = config.elevation_std * config.elevation_std;
  covariance(2, 2) = config.range_std * config.range_std;
  filter.updateBearingRange(measurement, covariance);
}

}  // namespace

MonteCarloResult runConsistencyMonteCarlo(const MonteCarloConfig& config) {
  if (config.runs < 1 || config.dt <= 0.0 || config.duration <= 0.0) {
    throw std::invalid_argument("Monte Carlo config needs positive runs, dt, and duration");
  }
  const int steps = std::max(1, static_cast<int>(std::lround(config.duration / config.dt)));
  MonteCarloResult result;
  result.runs = config.runs;
  result.dt = config.dt;
  result.state_dim = config.model == TargetKalmanFilter::Model::ConstantAcceleration ? 9 : 6;
  result.time.assign(steps, 0.0);
  result.mean_nees.assign(steps, 0.0);
  result.mean_nis.assign(steps, 0.0);
  result.pos_rmse.assign(steps, 0.0);
  result.vel_rmse.assign(steps, 0.0);

  for (int run = 0; run < config.runs; ++run) {
    GaussianNoise noise(config.seed + static_cast<unsigned>(run) * 10007u);
    TargetKalmanFilter filter(config.model, config.accel_std, config.jerk_std);
    TargetState prior;
    prior.position = config.prior_position;
    prior.velocity = config.prior_velocity;
    prior.acceleration = config.prior_acceleration;
    filter.initialize(prior, config.pos_var, config.vel_var, config.acc_var);

    TargetState truth;
    samplePrior(truth, config, noise);

    for (int k = 0; k < steps; ++k) {
      if (k > 0) {
        propagateTruth(truth, config, noise);
        filter.predict(config.dt);
      }
      updateFilter(filter, truth, config, noise);
      const TargetState estimate = filter.estimate();
      const double pos_sq = (estimate.position - truth.position).squaredNorm();
      const double vel_sq = (estimate.velocity - truth.velocity).squaredNorm();
      result.mean_nees[k] += filter.nees(truth);
      result.mean_nis[k] += filter.lastNis();
      result.pos_rmse[k] += pos_sq;
      result.vel_rmse[k] += vel_sq;
      result.time[k] = k * config.dt;
    }
  }

  const double inv_runs = 1.0 / static_cast<double>(config.runs);
  double nees_sum = 0.0;
  double nis_sum = 0.0;
  for (int k = 0; k < steps; ++k) {
    result.mean_nees[k] *= inv_runs;
    result.mean_nis[k] *= inv_runs;
    result.pos_rmse[k] = std::sqrt(result.pos_rmse[k] * inv_runs);
    result.vel_rmse[k] = std::sqrt(result.vel_rmse[k] * inv_runs);
    nees_sum += result.mean_nees[k];
    nis_sum += result.mean_nis[k];
  }
  result.average_nees = nees_sum / static_cast<double>(steps);
  result.average_nis = nis_sum / static_cast<double>(steps);
  return result;
}

}  // namespace gimbal_tracker
