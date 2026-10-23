#include "gimbal_tracker/gimbal_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using namespace gimbal_tracker;

Eigen::Matrix<double, 7, 1> turnState(double x, double y, double z, double vx, double vy, double vz,
                                      double omega) {
  Eigen::Matrix<double, 7, 1> state;
  state << x, y, z, vx, vy, vz, omega;
  return state;
}

EstimatorConfig immConfig() {
  EstimatorConfig config;
  config.model = "imm";
  config.accel_std = 0.4;
  config.jerk_std = 1.0;
  config.ct_accel_std = 0.3;
  config.turn_rate_std = 0.15;
  config.p0_pos = 4.0;
  config.p0_vel = 25.0;
  config.p0_acc = 4.0;
  config.p0_turn_rate = 0.05;
  config.imm_sojourn_cv_s = 6.0;
  config.imm_sojourn_ca_s = 3.0;
  config.imm_sojourn_ct_s = 4.0;
  config.imm_mu_cv = 0.5;
  config.imm_mu_ca = 0.25;
  config.imm_mu_ct = 0.25;
  return config;
}

}  // namespace

TEST(CoordinatedTurn, PredictMatchesTheClosedForm) {
  const double omega = 0.2;
  const double speed = 10.0;
  const double dt = 0.5;
  const auto next = predictCoordinatedTurn(turnState(0, 0, 3, speed, 0, 0, omega), dt);
  EXPECT_NEAR(next(0), (speed / omega) * std::sin(omega * dt), 1e-9);
  EXPECT_NEAR(next(1), (speed / omega) * (1.0 - std::cos(omega * dt)), 1e-9);
  EXPECT_NEAR(next(2), 3.0, 1e-12);
  EXPECT_NEAR(next(3), speed * std::cos(omega * dt), 1e-9);
  EXPECT_NEAR(next(4), speed * std::sin(omega * dt), 1e-9);
  EXPECT_NEAR(next(5), 0.0, 1e-12);
  EXPECT_NEAR(next(6), omega, 1e-12);
  EXPECT_NEAR(std::hypot(next(3), next(4)), speed, 1e-9);
}

TEST(CoordinatedTurn, TinyTurnRateMatchesStraightFlight) {
  const double dt = 0.2;
  const auto turned = predictCoordinatedTurn(turnState(1, 2, 3, 4, -1, 0.5, 1e-8), dt);
  EXPECT_NEAR(turned(0), 1.0 + 4.0 * dt, 1e-6);
  EXPECT_NEAR(turned(1), 2.0 - 1.0 * dt, 1e-6);
  EXPECT_NEAR(turned(2), 3.0 + 0.5 * dt, 1e-9);
  EXPECT_NEAR(turned(3), 4.0, 1e-5);
  EXPECT_NEAR(turned(4), -1.0, 1e-5);
}

TEST(CoordinatedTurn, JacobianMatchesFiniteDifference) {
  const auto state = turnState(5, -2, 1, 8, 3, -0.2, 0.25);
  const double dt = 0.1;
  const auto jacobian = coordinatedTurnJacobian(state, dt);
  const double eps = 1e-6;
  for (int column = 0; column < 7; ++column) {
    auto plus = state;
    auto minus = state;
    plus(column) += eps;
    minus(column) -= eps;
    const auto numeric = (predictCoordinatedTurn(plus, dt) - predictCoordinatedTurn(minus, dt)) / (2.0 * eps);
    for (int row = 0; row < 7; ++row) {
      EXPECT_NEAR(jacobian(row, column), numeric(row), 1e-5) << "row " << row << " col " << column;
    }
  }
}

TEST(CoordinatedTurnFilter, MeanFollowsTheDeterministicTurn) {
  CoordinatedTurnFilter filter(0.0, 0.0);
  TargetState prior;
  prior.position = {0, 0, 0};
  prior.velocity = {12, 0, 0};
  filter.initialize(prior, 1.0, 1.0, 0.01);
  filter.predict(0.4);
  const auto expected = predictCoordinatedTurn(turnState(0, 0, 0, 12, 0, 0, 0), 0.4);
  EXPECT_NEAR(filter.state()(0), expected(0), 1e-9);
  EXPECT_NEAR(filter.state()(3), expected(3), 1e-9);
}

TEST(ImmMixing, WeightsAndSharedStateFollowTheMarkovUpdate) {
  std::array<GaussianMoment, 3> modes;
  for (int mode = 0; mode < 3; ++mode) {
    const int n = mode == 1 ? 9 : (mode == 2 ? 7 : 6);
    modes[static_cast<std::size_t>(mode)].mean = Eigen::VectorXd::Zero(n);
    modes[static_cast<std::size_t>(mode)].covariance = Eigen::MatrixXd::Identity(n, n);
  }
  modes[0].mean(0) = 0.0;
  modes[1].mean(0) = 10.0;
  modes[1].mean(6) = 2.0;
  modes[2].mean(0) = 20.0;
  modes[2].mean(6) = 0.4;

  Eigen::Vector3d probability;
  probability << 0.5, 0.3, 0.2;
  Eigen::Matrix3d transition;
  transition << 0.8, 0.1, 0.1, 0.2, 0.7, 0.1, 0.1, 0.1, 0.8;

  std::array<GaussianMoment, 3> mixed;
  Eigen::Matrix3d conditional;
  Eigen::Vector3d predicted;
  mixImmModes(modes, probability, transition, 4.0, 0.05, mixed, conditional, predicted);

  EXPECT_NEAR(predicted(0), 0.48, 1e-12);
  EXPECT_NEAR(predicted(1), 0.28, 1e-12);
  EXPECT_NEAR(predicted(2), 0.24, 1e-12);
  EXPECT_NEAR(conditional.col(0).sum(), 1.0, 1e-12);
  EXPECT_NEAR(conditional(0, 0), 0.8 * 0.5 / 0.48, 1e-12);
  EXPECT_NEAR(conditional(1, 0), 0.2 * 0.3 / 0.48, 1e-12);
  EXPECT_NEAR(conditional(2, 0), 0.1 * 0.2 / 0.48, 1e-12);
  const double mixed_x = conditional(0, 0) * 0.0 + conditional(1, 0) * 10.0 + conditional(2, 0) * 20.0;
  EXPECT_NEAR(mixed[0].mean(0), mixed_x, 1e-12);
  EXPECT_NEAR(mixed[1].mean(6), 2.0, 1e-12);
  EXPECT_NEAR(mixed[2].mean(6), 0.4, 1e-12);
  EXPECT_EQ(mixed[0].mean.size(), 6);
  EXPECT_EQ(mixed[1].mean.size(), 9);
  EXPECT_EQ(mixed[2].mean.size(), 7);
}

TEST(ImmFilter, ProbabilitiesStayADistributionAndATurnFavorsTheTurnModel) {
  InteractingMultipleModel filter(immConfig());
  TargetKalmanFilter cv(TargetKalmanFilter::Model::ConstantVelocity, 0.4, 1.0);
  const double dt = 0.05;
  const double omega = 0.4;
  const double speed = 15.0;
  Eigen::Matrix<double, 7, 1> truth = turnState(20, -10, 2, speed, 0, 0, omega);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 0.25;
  GaussianNoise noise(5);

  auto measure = [&](const Eigen::Matrix<double, 7, 1>& state) {
    Eigen::Vector3d z;
    z << state(0) + noise.sample(0.5), state(1) + noise.sample(0.5), state(2) + noise.sample(0.5);
    return z;
  };

  const Eigen::Vector3d z0 = measure(truth);
  filter.initializeFromCartesian(z0, 4.0, 25.0, 4.0);
  cv.initializeFromCartesian(z0, 4.0, 25.0, 4.0);
  EXPECT_NEAR(filter.modelProbability().sum(), 1.0, 1e-12);

  for (int step = 0; step < 80; ++step) {
    truth = predictCoordinatedTurn(truth, dt);
    filter.predict(dt);
    cv.predict(dt);
    const Eigen::Vector3d z = measure(truth);
    filter.updateCartesian(z, covariance);
    cv.updateCartesian(z, covariance);
    EXPECT_NEAR(filter.modelProbability().sum(), 1.0, 1e-9);
    EXPECT_GT(filter.modelProbability().minCoeff(), 0.0);
  }

  EXPECT_GT(filter.modelProbability()(2), filter.modelProbability()(0));
  EXPECT_GT(filter.modelProbability()(2), 0.4);
  const double imm_error = (filter.estimate().position - Vec3{truth(0), truth(1), truth(2)}).norm();
  const double cv_error = (cv.estimate().position - Vec3{truth(0), truth(1), truth(2)}).norm();
  EXPECT_LT(imm_error, cv_error);
  EXPECT_LT(imm_error, 3.0);
}

TEST(TargetManeuvers, TurnWeaveAndSequenceMatchTheirDefinitions) {
  TargetConfig config;
  config.trajectory = "coordinated_turn";
  config.position = {0, 0, 1};
  config.velocity = {10, 0, 0};
  config.turn_rate = 0.25;
  const TargetTrajectory turn(config);
  const TargetState at = turn.at(0.8);
  EXPECT_NEAR(at.velocity.norm(), 10.0, 1e-9);
  EXPECT_NEAR(at.position.y, (10.0 / 0.25) * (1.0 - std::cos(0.25 * 0.8)), 1e-8);

  config.trajectory = "weave";
  config.weave_amplitude = 0.3;
  config.weave_omega = 1.2;
  const TargetTrajectory weave(config);
  const TargetState start = weave.at(0.0);
  EXPECT_NEAR(start.position.x, 0.0, 1e-9);
  EXPECT_NEAR(start.velocity.x, 10.0, 1e-8);
  const TargetState later = weave.at(0.4);
  EXPECT_NEAR(std::hypot(later.velocity.x, later.velocity.y), 10.0, 1e-6);

  config.trajectory = "sequence";
  MotionSegment straight;
  straight.kind = "straight";
  straight.duration_s = 2.0;
  MotionSegment bend;
  bend.kind = "coordinated_turn";
  bend.duration_s = 1.0;
  bend.turn_rate = 0.2;
  config.segments = {straight, bend};
  config.speed_ramp_s = 1.0;
  const TargetTrajectory sequence(config);
  const TargetState coast = sequence.at(2.0);
  EXPECT_NEAR(coast.position.x, 20.0, 1e-8);
  EXPECT_NEAR(coast.velocity.y, 0.0, 1e-8);
  EXPECT_EQ(sequence.kindAt(1.0), "straight");
  EXPECT_EQ(sequence.kindAt(2.5), "coordinated_turn");
  EXPECT_NEAR(sequence.duration(), 3.0, 1e-12);
}
