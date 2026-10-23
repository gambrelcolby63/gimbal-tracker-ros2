#include "gimbal_tracker/gimbal_tracker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

using namespace gimbal_tracker;

TargetKalmanFilter makeFilter(TargetKalmanFilter::Model model, double accel_std = 1.0,
                              double jerk_std = 1.0) {
  return TargetKalmanFilter(model, accel_std, jerk_std);
}

}  // namespace

TEST(Kalman, CartesianUpdateMatchesScalarFormula) {
  TargetKalmanFilter filter = makeFilter(TargetKalmanFilter::Model::ConstantVelocity);
  filter.initialize(TargetState{}, 1.0, 10.0, 1.0);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  filter.updateCartesian(Eigen::Vector3d(1.0, 0.0, 0.0), covariance);
  EXPECT_NEAR(filter.estimate().position.x, 0.5, 1e-12);
  EXPECT_NEAR(filter.covariance()(0, 0), 0.5, 1e-12);
  EXPECT_NEAR(filter.estimate().velocity.x, 0.0, 1e-12);
  EXPECT_NEAR(filter.lastNis(), 0.5, 1e-12);
}

TEST(Kalman, PredictPropagatesConstantVelocityAndProcessNoise) {
  TargetKalmanFilter filter = makeFilter(TargetKalmanFilter::Model::ConstantVelocity, 1.5, 0.0);
  TargetState prior;
  prior.velocity = {1.0, -2.0, 0.5};
  filter.initialize(prior, 1e-12, 1e-12, 1.0);
  filter.predict(0.5);
  EXPECT_NEAR(filter.estimate().position.x, 0.5, 1e-9);
  EXPECT_NEAR(filter.estimate().position.y, -1.0, 1e-9);
  EXPECT_NEAR(filter.estimate().velocity.x, 1.0, 1e-12);

  TargetKalmanFilter noise = makeFilter(TargetKalmanFilter::Model::ConstantVelocity, 2.0, 0.0);
  noise.initialize(TargetState{}, 1e-18, 1e-18, 1.0);
  const double dt = 0.2;
  noise.predict(dt);
  const double q = 4.0;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  // F P F^T is negligible next to Q because P starts at 1e-18.
  EXPECT_NEAR(noise.covariance()(0, 0), q * dt4 / 4.0, 1e-9);
  EXPECT_NEAR(noise.covariance()(0, 3), q * dt3 / 2.0, 1e-9);
  EXPECT_NEAR(noise.covariance()(3, 3), q * dt2, 1e-9);
}

TEST(Kalman, PredictGrowsVelocityVarianceAndUpdateShrinksPositionVariance) {
  TargetKalmanFilter filter = makeFilter(TargetKalmanFilter::Model::ConstantVelocity, 1.0, 1.0);
  filter.initialize(TargetState{}, 1.0, 1.0, 1.0);
  const double vel_before = filter.covariance()(3, 3);
  filter.predict(0.1);
  EXPECT_GT(filter.covariance()(3, 3), vel_before);
  const double pos_before = filter.covariance()(0, 0);
  filter.updateCartesian(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.25);
  EXPECT_LT(filter.covariance()(0, 0), pos_before);
}

TEST(Kalman, ConstantVelocityNeesIsConsistent) {
  MonteCarloConfig config;
  config.model = TargetKalmanFilter::Model::ConstantVelocity;
  config.runs = 40;
  config.dt = 0.05;
  config.duration = 4.0;
  config.accel_std = 1.0;
  config.meas_std = 0.5;
  config.pos_var = 4.0;
  config.vel_var = 4.0;
  config.seed = 7;
  const MonteCarloResult result = runConsistencyMonteCarlo(config);
  EXPECT_NEAR(result.average_nees, 6.0, 1.0);
  EXPECT_NEAR(result.average_nis, 3.0, 0.5);
  EXPECT_LT(result.pos_rmse.back(), 1.5);
}

TEST(Kalman, ConstantAccelerationNeesIsConsistent) {
  MonteCarloConfig config;
  config.model = TargetKalmanFilter::Model::ConstantAcceleration;
  config.runs = 30;
  config.dt = 0.05;
  config.duration = 4.0;
  config.jerk_std = 1.0;
  config.meas_std = 0.5;
  config.pos_var = 4.0;
  config.vel_var = 4.0;
  config.acc_var = 4.0;
  config.seed = 11;
  const MonteCarloResult result = runConsistencyMonteCarlo(config);
  EXPECT_NEAR(result.average_nees, 9.0, 1.5);
  EXPECT_NEAR(result.average_nis, 3.0, 0.6);
}

TEST(Kalman, BearingJacobianMatchesFiniteDifference) {
  const Vec3 base{30.0, -12.0, 4.0};
  auto at = [](const Vec3& position) {
    TargetKalmanFilter filter(TargetKalmanFilter::Model::ConstantVelocity, 1.0, 1.0);
    TargetState prior;
    prior.position = position;
    filter.initialize(prior, 1.0, 1.0, 1.0);
    Eigen::Vector3d predicted;
    Eigen::MatrixXd jacobian;
    filter.bearingRangeModel(predicted, jacobian);
    return std::make_pair(predicted, jacobian);
  };

  const auto nominal = at(base);
  const double eps = 1e-5;
  for (int axis = 0; axis < 3; ++axis) {
    Vec3 plus = base;
    Vec3 minus = base;
    if (axis == 0) {
      plus.x += eps;
      minus.x -= eps;
    } else if (axis == 1) {
      plus.y += eps;
      minus.y -= eps;
    } else {
      plus.z += eps;
      minus.z -= eps;
    }
    const auto hp = at(plus);
    const auto hm = at(minus);
    Eigen::Vector3d numerical;
    numerical(0) = wrapPi(hp.first(0) - hm.first(0)) / (2.0 * eps);
    numerical(1) = (hp.first(1) - hm.first(1)) / (2.0 * eps);
    numerical(2) = (hp.first(2) - hm.first(2)) / (2.0 * eps);
    for (int row = 0; row < 3; ++row) {
      EXPECT_NEAR(nominal.second(row, axis), numerical(row), 1e-5);
    }
  }
}

TEST(Kalman, BearingUpdateCrossesTheAzimuthBranchCut) {
  TargetState truth;
  truth.position = {-40.0, 12.0, 1.0};
  truth.velocity = {0.0, -3.0, 0.0};
  TargetKalmanFilter filter(TargetKalmanFilter::Model::ConstantVelocity, 0.5, 1.0);
  filter.initialize(truth, 0.25, 1.0, 1.0);
  GaussianNoise noise(19);
  const double dt = 0.02;
  double max_error = 0.0;
  bool crossed = false;
  for (int k = 0; k < 400; ++k) {
    if (k > 0) {
      truth.position = truth.position + truth.velocity * dt;
      filter.predict(dt);
    }
    const LookAngles look = lookAngles(Vec3{}, truth.position);
    if (truth.position.y < 0.0) {
      crossed = true;
    }
    Eigen::Vector3d measurement;
    measurement << wrapPi(look.azimuth + noise.sample(0.001)),
        look.elevation + noise.sample(0.001), look.range + noise.sample(0.05);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = 1e-6;
    covariance(1, 1) = 1e-6;
    covariance(2, 2) = 0.0025;
    filter.updateBearingRange(measurement, covariance);
    max_error = std::max(max_error, (filter.estimate().position - truth.position).norm());
    EXPECT_TRUE(std::isfinite(filter.estimate().position.x));
  }
  EXPECT_TRUE(crossed);
  EXPECT_LT(max_error, 2.0);
}

TEST(Kalman, ConstantAccelerationTracksAManeuverTighterThanConstantVelocity) {
  const double dt = 0.05;
  const int steps = 80;
  GaussianNoise noise(3);
  TargetKalmanFilter cv(TargetKalmanFilter::Model::ConstantVelocity, 0.3, 1.0);
  TargetKalmanFilter ca(TargetKalmanFilter::Model::ConstantAcceleration, 0.3, 0.3);
  TargetState truth;
  truth.velocity = {4.0, 0.0, 0.0};
  truth.acceleration = {0.0, 0.8, 0.0};
  cv.initialize(truth, 1.0, 4.0, 4.0);
  ca.initialize(truth, 1.0, 4.0, 4.0);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 0.09;

  auto measure = [&](const TargetState& state) {
    Eigen::Vector3d z;
    z << state.position.x + noise.sample(0.3), state.position.y + noise.sample(0.3),
        state.position.z + noise.sample(0.3);
    return z;
  };

  cv.updateCartesian(measure(truth), covariance);
  // Independent noise draws would make the comparison unfair; replay one sequence.
  noise.reseed(3);
  ca.updateCartesian(measure(truth), covariance);

  for (int k = 1; k < steps; ++k) {
    truth.position = truth.position + truth.velocity * dt + truth.acceleration * (0.5 * dt * dt);
    truth.velocity = truth.velocity + truth.acceleration * dt;
    cv.predict(dt);
    ca.predict(dt);
    noise.reseed(1000u + static_cast<unsigned>(k));
    const Eigen::Vector3d z = measure(truth);
    cv.updateCartesian(z, covariance);
    ca.updateCartesian(z, covariance);
  }
  const double cv_error = (cv.estimate().position - truth.position).norm();
  const double ca_error = (ca.estimate().position - truth.position).norm();
  EXPECT_LT(ca_error, cv_error);
  EXPECT_LT(ca_error, 1.0);
}

TEST(TargetTrajectory, StraightLineAndCircle) {
  TargetConfig config;
  config.trajectory = "constant_velocity";
  config.position = {1.0, 2.0, 3.0};
  config.velocity = {4.0, -1.0, 0.5};
  const TargetTrajectory line(config);
  const TargetState at = line.at(2.0);
  EXPECT_NEAR(at.position.x, 9.0, 1e-12);
  EXPECT_NEAR(at.position.y, 0.0, 1e-12);
  EXPECT_NEAR(at.velocity.z, 0.5, 1e-12);
  EXPECT_NEAR(at.acceleration.norm(), 0.0, 1e-12);

  config.trajectory = "circle";
  config.circle_radius = 10.0;
  config.circle_omega = 0.3;
  config.circle_z = 2.0;
  const TargetTrajectory circle(config);
  const TargetState start = circle.at(0.0);
  EXPECT_NEAR(start.position.x, 10.0, 1e-12);
  EXPECT_NEAR(start.position.z, 2.0, 1e-12);
  EXPECT_NEAR(start.position.norm(), std::hypot(10.0, 2.0), 1e-12);
  EXPECT_NEAR(start.acceleration.norm(), 10.0 * 0.3 * 0.3, 1e-12);
}
