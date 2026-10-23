#include "gimbal_tracker/imm.hpp"

#include "gimbal_tracker/angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

constexpr double kTurnFloor = 1.0e-4;
constexpr double kRhoXyFloor = 1.0e-4;
constexpr int kCv = 0;
constexpr int kCa = 1;
constexpr int kCt = 2;

Eigen::VectorXd kinematic(const Eigen::VectorXd& state) {
  if (state.size() < 6) {
    throw std::invalid_argument("IMM mode state is shorter than 6");
  }
  return state.head(6);
}

template <typename Matrix>
void symmetrize(Matrix& covariance) {
  covariance = 0.5 * (covariance + covariance.transpose());
}

double logDetFromLlt(const Eigen::LLT<Eigen::MatrixXd>& llt) {
  const Eigen::MatrixXd lower = llt.matrixL();
  double sum = 0.0;
  for (int i = 0; i < lower.rows(); ++i) {
    sum += std::log(std::max(lower(i, i), 1.0e-300));
  }
  return 2.0 * sum;
}

Eigen::Matrix<double, 7, 1> asTurn(const Eigen::VectorXd& state) {
  Eigen::Matrix<double, 7, 1> turn = Eigen::Matrix<double, 7, 1>::Zero();
  turn.head(6) = kinematic(state);
  if (state.size() == 7) {
    turn(6) = state(6);
  }
  return turn;
}

void applyJoseph(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, double& nis, double& log_likelihood,
                 const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
  const Eigen::MatrixXd S = H * covariance * H.transpose() + R;
  const Eigen::LLT<Eigen::MatrixXd> llt(S);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("Innovation covariance is not positive definite");
  }
  const int nz = static_cast<int>(innovation.size());
  const int n = static_cast<int>(state.size());
  const Eigen::MatrixXd K = covariance * H.transpose() * llt.solve(Eigen::MatrixXd::Identity(nz, nz));
  state += K * innovation;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd I_KH = I - K * H;
  covariance = I_KH * covariance * I_KH.transpose() + K * R * K.transpose();
  symmetrize(covariance);
  nis = innovation.dot(llt.solve(innovation));
  const double log_det = logDetFromLlt(llt);
  log_likelihood = -0.5 * (nis + log_det + static_cast<double>(nz) * std::log(2.0 * kPi));
}

Eigen::MatrixXd whiteAccelerationNoise(int n, double accel_std, double dt) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
  const double q = accel_std * accel_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  for (int axis = 0; axis < 3; ++axis) {
    Q(axis, axis) = q * dt4 / 4.0;
    Q(axis, axis + 3) = q * dt3 / 2.0;
    Q(axis + 3, axis) = Q(axis, axis + 3);
    Q(axis + 3, axis + 3) = q * dt2;
  }
  return Q;
}

void bearingParts(const Eigen::Vector3d& position, int n, Eigen::Vector3d& predicted, Eigen::MatrixXd& H,
                  Eigen::Vector3d& innovation, Eigen::Matrix3d& R, const Eigen::Vector3d& measurement,
                  const Eigen::Matrix3d& covariance) {
  const double px = position(0);
  const double py = position(1);
  const double pz = position(2);
  const double rho_xy = std::hypot(px, py);
  const double rho = std::sqrt(px * px + py * py + pz * pz);
  predicted(0) = std::atan2(py, px);
  predicted(1) = std::atan2(pz, std::max(rho_xy, 0.0));
  predicted(2) = rho;
  H = Eigen::MatrixXd::Zero(3, n);
  if (rho >= 1.0e-9) {
    const double rho_xy_used = std::max(rho_xy, kRhoXyFloor);
    const double rho_xy2 = rho_xy_used * rho_xy_used;
    const double rho2 = rho * rho;
    if (rho_xy >= kRhoXyFloor) {
      H(0, 0) = -py / rho_xy2;
      H(0, 1) = px / rho_xy2;
      H(1, 0) = -px * pz / (rho2 * rho_xy);
      H(1, 1) = -py * pz / (rho2 * rho_xy);
    }
    H(1, 2) = rho_xy_used / rho2;
    H(2, 0) = px / rho;
    H(2, 1) = py / rho;
    H(2, 2) = pz / rho;
  }
  innovation(0) = wrapPi(measurement(0) - predicted(0));
  innovation(1) = measurement(1) - predicted(1);
  innovation(2) = measurement(2) - predicted(2);
  R = covariance;
  if (rho_xy < kRhoXyFloor) {
    H.row(0).setZero();
    innovation(0) = 0.0;
    R.row(0).setZero();
    R.col(0).setZero();
    R(0, 0) = 1.0e6;
  }
}

GaussianMoment momentFromKin(const Eigen::VectorXd& kin, const Eigen::MatrixXd& kin_p, int extra,
                             double extra_variance) {
  GaussianMoment moment;
  const int n = 6 + extra;
  moment.mean = Eigen::VectorXd::Zero(n);
  moment.mean.head(6) = kin;
  moment.covariance = Eigen::MatrixXd::Zero(n, n);
  moment.covariance.topLeftCorner(6, 6) = kin_p;
  for (int i = 0; i < extra; ++i) {
    moment.covariance(6 + i, 6 + i) = extra_variance;
  }
  return moment;
}

}  // namespace

Eigen::Matrix<double, 7, 1> predictCoordinatedTurn(const Eigen::Matrix<double, 7, 1>& state, double dt) {
  Eigen::Matrix<double, 7, 1> next = state;
  if (!(dt > 0.0)) {
    return next;
  }
  const double vx = state(3);
  const double vy = state(4);
  const double vz = state(5);
  const double omega = state(6);
  if (std::abs(omega) < kTurnFloor) {
    next(0) += vx * dt - 0.5 * vy * omega * dt * dt;
    next(1) += vy * dt + 0.5 * vx * omega * dt * dt;
    next(2) += vz * dt;
    next(3) = vx - vy * omega * dt;
    next(4) = vy + vx * omega * dt;
    next(5) = vz;
    return next;
  }
  const double s = std::sin(omega * dt);
  const double c = std::cos(omega * dt);
  next(0) += (vx * s - vy * (1.0 - c)) / omega;
  next(1) += (vx * (1.0 - c) + vy * s) / omega;
  next(2) += vz * dt;
  next(3) = vx * c - vy * s;
  next(4) = vx * s + vy * c;
  next(5) = vz;
  next(6) = omega;
  return next;
}

Eigen::Matrix<double, 7, 7> coordinatedTurnJacobian(const Eigen::Matrix<double, 7, 1>& state, double dt) {
  Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity();
  if (!(dt > 0.0)) {
    return F;
  }
  const double vx = state(3);
  const double vy = state(4);
  const double omega = state(6);
  if (std::abs(omega) < kTurnFloor) {
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(0, 3) = dt;
    F(0, 4) = -0.5 * omega * dt * dt;
    F(1, 3) = 0.5 * omega * dt * dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(0, 6) = -0.5 * vy * dt * dt;
    F(1, 6) = 0.5 * vx * dt * dt;
    F(3, 4) = -omega * dt;
    F(4, 3) = omega * dt;
    F(3, 6) = -vy * dt;
    F(4, 6) = vx * dt;
    return F;
  }
  const double s = std::sin(omega * dt);
  const double c = std::cos(omega * dt);
  F(0, 3) = s / omega;
  F(0, 4) = -(1.0 - c) / omega;
  F(1, 3) = (1.0 - c) / omega;
  F(1, 4) = s / omega;
  F(2, 5) = dt;
  F(3, 3) = c;
  F(3, 4) = -s;
  F(4, 3) = s;
  F(4, 4) = c;
  const double along = vx * s - vy * (1.0 - c);
  const double cross = vx * (1.0 - c) + vy * s;
  const double d_along = dt * (vx * c - vy * s);
  const double d_cross = dt * (vx * s + vy * c);
  F(0, 6) = (d_along * omega - along) / (omega * omega);
  F(1, 6) = (d_cross * omega - cross) / (omega * omega);
  F(3, 6) = -dt * (vx * s + vy * c);
  F(4, 6) = dt * (vx * c - vy * s);
  return F;
}

Eigen::Matrix3d immTransition(const Eigen::Vector3d& sojourn_s, double dt) {
  Eigen::Matrix3d transition = Eigen::Matrix3d::Zero();
  const double step = std::max(dt, 0.0);
  for (int from = 0; from < 3; ++from) {
    const double sojourn = std::max(sojourn_s(from), 1.0e-6);
    const double stay = std::exp(-step / sojourn);
    const double leave = 0.5 * (1.0 - stay);
    for (int to = 0; to < 3; ++to) {
      transition(from, to) = (from == to) ? stay : leave;
    }
  }
  return transition;
}

void mixImmModes(const std::array<GaussianMoment, 3>& modes, const Eigen::Vector3d& probability,
                 const Eigen::Matrix3d& transition, double acceleration_variance,
                 double turn_rate_variance, std::array<GaussianMoment, 3>& mixed,
                 Eigen::Matrix3d& conditional_probability, Eigen::Vector3d& predicted_probability) {
  for (int mode = 0; mode < 3; ++mode) {
    const int expected = mode == kCa ? 9 : (mode == kCt ? 7 : 6);
    if (modes[static_cast<std::size_t>(mode)].mean.size() != expected) {
      throw std::invalid_argument("IMM mix expected CV/CA/CT state sizes 6/9/7");
    }
  }
  predicted_probability = transition.transpose() * probability;
  for (int to = 0; to < 3; ++to) {
    predicted_probability(to) = std::max(predicted_probability(to), 1.0e-300);
  }
  predicted_probability /= predicted_probability.sum();

  Eigen::Matrix<double, 6, 1> mixed_kin = Eigen::Matrix<double, 6, 1>::Zero();
  std::array<Eigen::Matrix<double, 6, 1>, 3> kin_mean;
  std::array<Eigen::Matrix<double, 6, 6>, 3> kin_cov;
  conditional_probability.setZero();
  for (int from = 0; from < 3; ++from) {
    kin_mean[static_cast<std::size_t>(from)] =
        modes[static_cast<std::size_t>(from)].mean.head(6);
    kin_cov[static_cast<std::size_t>(from)] =
        modes[static_cast<std::size_t>(from)].covariance.topLeftCorner(6, 6);
  }

  for (int to = 0; to < 3; ++to) {
    double norm = 0.0;
    for (int from = 0; from < 3; ++from) {
      norm += transition(from, to) * probability(from);
    }
    norm = std::max(norm, 1.0e-300);
    Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
    for (int from = 0; from < 3; ++from) {
      const double weight = transition(from, to) * probability(from) / norm;
      conditional_probability(from, to) = weight;
      mean += weight * kin_mean[static_cast<std::size_t>(from)];
    }
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    for (int from = 0; from < 3; ++from) {
      const double weight = conditional_probability(from, to);
      const Eigen::Matrix<double, 6, 1> delta = kin_mean[static_cast<std::size_t>(from)] - mean;
      covariance += weight * (kin_cov[static_cast<std::size_t>(from)] + delta * delta.transpose());
    }
    symmetrize(covariance);
    mixed_kin = mean;
    const double acc_var = std::max(acceleration_variance, 1.0e-8);
    const double omega_var = std::max(turn_rate_variance, 1.0e-8);
    if (to == kCv) {
      mixed[static_cast<std::size_t>(to)] = momentFromKin(mixed_kin, covariance, 0, 0.0);
    } else if (to == kCa) {
      mixed[static_cast<std::size_t>(to)] = momentFromKin(mixed_kin, covariance, 3, acc_var);
      mixed[static_cast<std::size_t>(to)].mean.tail(3) = modes[kCa].mean.tail(3);
      mixed[static_cast<std::size_t>(to)].covariance.bottomRightCorner(3, 3) =
          modes[kCa].covariance.bottomRightCorner(3, 3);
    } else {
      mixed[static_cast<std::size_t>(to)] = momentFromKin(mixed_kin, covariance, 1, omega_var);
      mixed[static_cast<std::size_t>(to)].mean(6) = modes[kCt].mean(6);
      mixed[static_cast<std::size_t>(to)].covariance(6, 6) = modes[kCt].covariance(6, 6);
    }
    symmetrize(mixed[static_cast<std::size_t>(to)].covariance);
  }
}

CoordinatedTurnFilter::CoordinatedTurnFilter(double accel_std, double turn_rate_std)
    : accel_std_(accel_std),
      turn_rate_std_(turn_rate_std),
      x_(Eigen::VectorXd::Zero(7)),
      P_(Eigen::MatrixXd::Identity(7, 7)) {}

void CoordinatedTurnFilter::initialize(const TargetState& prior, double pos_var, double vel_var,
                                       double turn_rate_var) {
  if (pos_var <= 0.0 || vel_var <= 0.0 || turn_rate_var <= 0.0) {
    throw std::invalid_argument("Turn-filter prior variances must be positive");
  }
  x_.setZero();
  P_.setZero();
  x_(0) = prior.position.x;
  x_(1) = prior.position.y;
  x_(2) = prior.position.z;
  x_(3) = prior.velocity.x;
  x_(4) = prior.velocity.y;
  x_(5) = prior.velocity.z;
  P_(0, 0) = P_(1, 1) = P_(2, 2) = pos_var;
  P_(3, 3) = P_(4, 4) = P_(5, 5) = vel_var;
  P_(6, 6) = turn_rate_var;
  initialized_ = true;
  last_nis_ = 0.0;
  last_log_likelihood_ = 0.0;
}

void CoordinatedTurnFilter::initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var,
                                                    double vel_var, double turn_rate_var) {
  TargetState prior;
  prior.position = {measurement(0), measurement(1), measurement(2)};
  initialize(prior, pos_var, vel_var, turn_rate_var);
}

void CoordinatedTurnFilter::initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var,
                                                       double vel_var, double turn_rate_var) {
  const double range = std::max(0.0, measurement(2));
  const double ce = std::cos(measurement(1));
  TargetState prior;
  prior.position = {range * ce * std::cos(measurement(0)), range * ce * std::sin(measurement(0)),
                    range * std::sin(measurement(1))};
  initialize(prior, pos_var, vel_var, turn_rate_var);
}

void CoordinatedTurnFilter::reset(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
  if (mean.size() != 7 || covariance.rows() != 7 || covariance.cols() != 7) {
    throw std::invalid_argument("Turn-filter reset dimension mismatch");
  }
  x_ = mean;
  P_ = covariance;
  symmetrize(P_);
  initialized_ = true;
  last_nis_ = 0.0;
}

Eigen::Matrix<double, 7, 1> CoordinatedTurnFilter::mean7() const { return asTurn(x_); }

void CoordinatedTurnFilter::predict(double dt) {
  if (!initialized_) {
    throw std::logic_error("Turn-filter predict called before initialize");
  }
  if (!(dt > 0.0)) {
    return;
  }
  const Eigen::Matrix<double, 7, 1> mean = mean7();
  const Eigen::Matrix<double, 7, 7> F = coordinatedTurnJacobian(mean, dt);
  x_ = predictCoordinatedTurn(mean, dt);
  Eigen::MatrixXd Q = whiteAccelerationNoise(7, accel_std_, dt);
  Q(6, 6) += turn_rate_std_ * turn_rate_std_ * dt;
  P_ = F * P_ * F.transpose() + Q;
  symmetrize(P_);
}

void CoordinatedTurnFilter::applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H,
                                        const Eigen::MatrixXd& R) {
  applyJoseph(x_, P_, last_nis_, last_log_likelihood_, innovation, H, R);
}

void CoordinatedTurnFilter::updateCartesian(const Eigen::Vector3d& measurement,
                                            const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("Turn-filter update called before initialize");
  }
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
  H.block<3, 3>(0, 0).setIdentity();
  applyUpdate(measurement - x_.head(3), H, covariance);
}

void CoordinatedTurnFilter::updateBearingRange(const Eigen::Vector3d& measurement,
                                               const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("Turn-filter update called before initialize");
  }
  Eigen::Vector3d predicted;
  Eigen::MatrixXd H;
  Eigen::Vector3d innovation;
  Eigen::Matrix3d R;
  bearingParts(x_.head(3), 7, predicted, H, innovation, R, measurement, covariance);
  applyUpdate(innovation, H, R);
}

TargetState CoordinatedTurnFilter::estimate() const {
  TargetState state;
  state.position = {x_(0), x_(1), x_(2)};
  state.velocity = {x_(3), x_(4), x_(5)};
  const double omega = x_(6);
  state.acceleration = {-omega * state.velocity.y, omega * state.velocity.x, 0.0};
  return state;
}

InteractingMultipleModel::InteractingMultipleModel(const EstimatorConfig& config)
    : config_(config),
      cv_(TargetKalmanFilter::Model::ConstantVelocity, config.accel_std, config.jerk_std),
      ca_(TargetKalmanFilter::Model::ConstantAcceleration, config.accel_std, config.jerk_std),
      ct_(config.ct_accel_std, config.turn_rate_std) {
  probability_ << config.imm_mu_cv, config.imm_mu_ca, config.imm_mu_ct;
  if (probability_.minCoeff() < 0.0 || !(probability_.sum() > 0.0)) {
    throw std::invalid_argument("IMM initial probabilities must be non-negative and not all zero");
  }
  probability_ /= probability_.sum();
  predicted_probability_ = probability_;
  combined_p_.setIdentity();
}

void InteractingMultipleModel::seedFilters(const TargetState& prior, double pos_var, double vel_var,
                                          double acc_var) {
  cv_.initialize(prior, pos_var, vel_var, acc_var);
  ca_.initialize(prior, pos_var, vel_var, acc_var);
  ct_.initialize(prior, pos_var, vel_var, std::max(config_.p0_turn_rate, 1.0e-8));
  probability_ << config_.imm_mu_cv, config_.imm_mu_ca, config_.imm_mu_ct;
  probability_ /= probability_.sum();
  predicted_probability_ = probability_;
  initialized_ = true;
  combine();
}

void InteractingMultipleModel::initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var,
                                                      double vel_var, double acc_var) {
  TargetState prior;
  prior.position = {measurement(0), measurement(1), measurement(2)};
  seedFilters(prior, pos_var, vel_var, acc_var);
}

void InteractingMultipleModel::initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var,
                                                         double vel_var, double acc_var) {
  const double range = std::max(0.0, measurement(2));
  const double ce = std::cos(measurement(1));
  TargetState prior;
  prior.position = {range * ce * std::cos(measurement(0)), range * ce * std::sin(measurement(0)),
                    range * std::sin(measurement(1))};
  seedFilters(prior, pos_var, vel_var, acc_var);
}

std::array<GaussianMoment, 3> InteractingMultipleModel::moments() const {
  std::array<GaussianMoment, 3> modes;
  modes[kCv] = {cv_.state(), cv_.covariance()};
  modes[kCa] = {ca_.state(), ca_.covariance()};
  modes[kCt] = {ct_.state(), ct_.covariance()};
  return modes;
}

void InteractingMultipleModel::mixAndPredict(double dt) {
  const Eigen::Vector3d sojourn(config_.imm_sojourn_cv_s, config_.imm_sojourn_ca_s, config_.imm_sojourn_ct_s);
  const Eigen::Matrix3d transition = immTransition(sojourn, dt);
  std::array<GaussianMoment, 3> mixed;
  Eigen::Matrix3d conditional = Eigen::Matrix3d::Zero();
  mixImmModes(moments(), probability_, transition, config_.p0_acc, config_.p0_turn_rate, mixed, conditional,
              predicted_probability_);
  cv_.reset(mixed[kCv].mean, mixed[kCv].covariance);
  ca_.reset(mixed[kCa].mean, mixed[kCa].covariance);
  ct_.reset(mixed[kCt].mean, mixed[kCt].covariance);
  cv_.predict(dt);
  ca_.predict(dt);
  ct_.predict(dt);
  probability_ = predicted_probability_;
  combine();
}

void InteractingMultipleModel::predict(double dt) {
  if (!initialized_) {
    throw std::logic_error("IMM predict called before initialize");
  }
  if (!(dt > 0.0)) {
    return;
  }
  mixAndPredict(dt);
}

void InteractingMultipleModel::absorbLikelihoods(const Eigen::Vector3d& prior,
                                                 const Eigen::Vector3d& log_likelihood) {
  Eigen::Vector3d log_weight;
  for (int mode = 0; mode < 3; ++mode) {
    log_weight(mode) = std::log(std::max(prior(mode), 1.0e-300)) + log_likelihood(mode);
  }
  const double pivot = log_weight.maxCoeff();
  double sum = 0.0;
  for (int mode = 0; mode < 3; ++mode) {
    sum += std::exp(log_weight(mode) - pivot);
  }
  for (int mode = 0; mode < 3; ++mode) {
    probability_(mode) = std::exp(log_weight(mode) - pivot) / sum;
  }
}

void InteractingMultipleModel::updateCartesian(const Eigen::Vector3d& measurement,
                                              const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("IMM update called before initialize");
  }
  const Eigen::Vector3d prior = probability_;
  cv_.updateCartesian(measurement, covariance);
  ca_.updateCartesian(measurement, covariance);
  ct_.updateCartesian(measurement, covariance);
  const Eigen::Vector3d log_likelihood(cv_.lastLogLikelihood(), ca_.lastLogLikelihood(),
                                       ct_.lastLogLikelihood());
  absorbLikelihoods(prior, log_likelihood);
  combine();
}

void InteractingMultipleModel::updateBearingRange(const Eigen::Vector3d& measurement,
                                                 const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("IMM update called before initialize");
  }
  const Eigen::Vector3d prior = probability_;
  cv_.updateBearingRange(measurement, covariance);
  ca_.updateBearingRange(measurement, covariance);
  ct_.updateBearingRange(measurement, covariance);
  const Eigen::Vector3d log_likelihood(cv_.lastLogLikelihood(), ca_.lastLogLikelihood(),
                                       ct_.lastLogLikelihood());
  absorbLikelihoods(prior, log_likelihood);
  combine();
}

void InteractingMultipleModel::combine() {
  const Eigen::Vector3d& weight = probability_;
  Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
  const std::array<Eigen::VectorXd, 3> states{cv_.state(), ca_.state(), ct_.state()};
  const std::array<Eigen::MatrixXd, 3> covs{cv_.covariance(), ca_.covariance(), ct_.covariance()};
  for (int mode = 0; mode < 3; ++mode) {
    mean += weight(mode) * states[static_cast<std::size_t>(mode)].head(6);
  }
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  for (int mode = 0; mode < 3; ++mode) {
    const Eigen::Matrix<double, 6, 1> delta = states[static_cast<std::size_t>(mode)].head(6) - mean;
    covariance += weight(mode) * (covs[static_cast<std::size_t>(mode)].topLeftCorner(6, 6) +
                                  delta * delta.transpose());
  }
  symmetrize(covariance);
  combined_p_ = covariance;
  combined_.position = {mean(0), mean(1), mean(2)};
  combined_.velocity = {mean(3), mean(4), mean(5)};
  const Vec3 ca_acc = ca_.estimate().acceleration;
  const double omega = ct_.state()(6);
  const Vec3 turn_acc{-omega * ct_.state()(4), omega * ct_.state()(3), 0.0};
  combined_.acceleration = weight(kCa) * ca_acc + weight(kCt) * turn_acc;
  last_nis_ = weight(kCv) * cv_.lastNis() + weight(kCa) * ca_.lastNis() + weight(kCt) * ct_.lastNis();
}

TargetState InteractingMultipleModel::estimate() const { return combined_; }

double InteractingMultipleModel::nees(const TargetState& truth) const {
  Eigen::Matrix<double, 6, 1> error;
  error << truth.position.x - combined_.position.x, truth.position.y - combined_.position.y,
      truth.position.z - combined_.position.z, truth.velocity.x - combined_.velocity.x,
      truth.velocity.y - combined_.velocity.y, truth.velocity.z - combined_.velocity.z;
  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(combined_p_);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  return error.dot(llt.solve(error));
}

TargetEstimator::TargetEstimator(const EstimatorConfig& config)
    : model_(config.model), turn_rate_prior_(std::max(config.p0_turn_rate, 1.0e-8)), view_p_(6, 6) {
  view_p_.setIdentity();
  if (model_ == "cv" || model_ == "ca") {
    kalman_ = std::make_unique<TargetKalmanFilter>(TargetKalmanFilter::modelFromString(model_), config.accel_std,
                                                   config.jerk_std);
  } else if (model_ == "ct") {
    turn_ = std::make_unique<CoordinatedTurnFilter>(config.ct_accel_std, config.turn_rate_std);
  } else if (model_ == "imm") {
    imm_ = std::make_unique<InteractingMultipleModel>(config);
  } else {
    throw std::invalid_argument("Estimator model must be cv, ca, ct, or imm");
  }
}

void TargetEstimator::initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                                             double acc_var) {
  if (kalman_) {
    kalman_->initializeFromCartesian(measurement, pos_var, vel_var, acc_var);
  } else if (turn_) {
    turn_->initializeFromCartesian(measurement, pos_var, vel_var, turn_rate_prior_);
  } else {
    imm_->initializeFromCartesian(measurement, pos_var, vel_var, acc_var);
  }
  refreshView();
}

void TargetEstimator::initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var,
                                                double vel_var, double acc_var) {
  if (kalman_) {
    kalman_->initializeFromBearingRange(measurement, pos_var, vel_var, acc_var);
  } else if (turn_) {
    turn_->initializeFromBearingRange(measurement, pos_var, vel_var, turn_rate_prior_);
  } else {
    imm_->initializeFromBearingRange(measurement, pos_var, vel_var, acc_var);
  }
  refreshView();
}

void TargetEstimator::predict(double dt) {
  if (kalman_) {
    kalman_->predict(dt);
  } else if (turn_) {
    turn_->predict(dt);
  } else {
    imm_->predict(dt);
  }
  refreshView();
}

void TargetEstimator::updateCartesian(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance) {
  if (kalman_) {
    kalman_->updateCartesian(measurement, covariance);
  } else if (turn_) {
    turn_->updateCartesian(measurement, covariance);
  } else {
    imm_->updateCartesian(measurement, covariance);
  }
  refreshView();
}

void TargetEstimator::updateBearingRange(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance) {
  if (kalman_) {
    kalman_->updateBearingRange(measurement, covariance);
  } else if (turn_) {
    turn_->updateBearingRange(measurement, covariance);
  } else {
    imm_->updateBearingRange(measurement, covariance);
  }
  refreshView();
}

bool TargetEstimator::initialized() const {
  if (kalman_) {
    return kalman_->initialized();
  }
  if (turn_) {
    return turn_->initialized();
  }
  return imm_->initialized();
}

TargetState TargetEstimator::estimate() const {
  if (kalman_) {
    return kalman_->estimate();
  }
  if (turn_) {
    return turn_->estimate();
  }
  return imm_->estimate();
}

const Eigen::MatrixXd& TargetEstimator::covariance() const { return view_p_; }

Eigen::Vector3d TargetEstimator::modelProbability() const {
  if (imm_) {
    return imm_->modelProbability();
  }
  Eigen::Vector3d probability = Eigen::Vector3d::Zero();
  if (model_ == "ca") {
    probability(1) = 1.0;
  } else if (model_ == "ct") {
    probability(2) = 1.0;
  } else {
    probability(0) = 1.0;
  }
  return probability;
}

double TargetEstimator::lastNis() const { return view_nis_; }

double TargetEstimator::nees(const TargetState& truth) const {
  if (kalman_) {
    return kalman_->nees(truth);
  }
  if (turn_) {
    Eigen::Matrix<double, 6, 1> error;
    const TargetState estimate = turn_->estimate();
    error << truth.position.x - estimate.position.x, truth.position.y - estimate.position.y,
        truth.position.z - estimate.position.z, truth.velocity.x - estimate.velocity.x,
        truth.velocity.y - estimate.velocity.y, truth.velocity.z - estimate.velocity.z;
    const Eigen::Matrix<double, 6, 6> P = turn_->covariance().topLeftCorner(6, 6);
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(P);
    if (llt.info() != Eigen::Success) {
      return std::numeric_limits<double>::infinity();
    }
    return error.dot(llt.solve(error));
  }
  return imm_->nees(truth);
}

void TargetEstimator::refreshView() {
  if (!initialized()) {
    return;
  }
  if (kalman_) {
    view_p_ = kalman_->covariance().topLeftCorner(6, 6);
    view_nis_ = kalman_->lastNis();
  } else if (turn_) {
    view_p_ = turn_->covariance().topLeftCorner(6, 6);
    view_nis_ = turn_->lastNis();
  } else {
    view_p_ = imm_->kinematicCovariance();
    view_nis_ = imm_->lastNis();
  }
}

}  // namespace gimbal_tracker
