#include "gimbal_tracker/kalman.hpp"

#include "gimbal_tracker/angle.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

constexpr double kRhoXyFloor = 1.0e-4;

}  // namespace

TargetKalmanFilter::TargetKalmanFilter(Model model, double accel_std, double jerk_std)
    : model_(model),
      n_(model == Model::ConstantAcceleration ? 9 : 6),
      accel_std_(accel_std),
      jerk_std_(jerk_std),
      x_(Eigen::VectorXd::Zero(n_)),
      P_(Eigen::MatrixXd::Identity(n_, n_)) {}

TargetKalmanFilter::Model TargetKalmanFilter::modelFromString(const std::string& name) {
  if (name == "cv") {
    return Model::ConstantVelocity;
  }
  if (name == "ca") {
    return Model::ConstantAcceleration;
  }
  throw std::invalid_argument("Kalman model must be 'cv' or 'ca', got '" + name + "'");
}

void TargetKalmanFilter::initialize(const TargetState& prior_mean, double pos_var, double vel_var,
                                    double acc_var) {
  if (pos_var <= 0.0 || vel_var <= 0.0 || (n_ == 9 && acc_var <= 0.0)) {
    throw std::invalid_argument("Kalman prior variances must be positive");
  }
  x_.setZero();
  P_.setZero();
  x_(0) = prior_mean.position.x;
  x_(1) = prior_mean.position.y;
  x_(2) = prior_mean.position.z;
  x_(3) = prior_mean.velocity.x;
  x_(4) = prior_mean.velocity.y;
  x_(5) = prior_mean.velocity.z;
  P_(0, 0) = P_(1, 1) = P_(2, 2) = pos_var;
  P_(3, 3) = P_(4, 4) = P_(5, 5) = vel_var;
  if (n_ == 9) {
    x_(6) = prior_mean.acceleration.x;
    x_(7) = prior_mean.acceleration.y;
    x_(8) = prior_mean.acceleration.z;
    P_(6, 6) = P_(7, 7) = P_(8, 8) = acc_var;
  }
  initialized_ = true;
  last_nis_ = 0.0;
}

void TargetKalmanFilter::initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var,
                                                 double vel_var, double acc_var) {
  TargetState prior;
  prior.position = {measurement(0), measurement(1), measurement(2)};
  initialize(prior, pos_var, vel_var, acc_var);
}

void TargetKalmanFilter::initializeFromBearingRange(const Eigen::Vector3d& measurement,
                                                    double pos_var, double vel_var, double acc_var) {
  const double azimuth = measurement(0);
  const double elevation = measurement(1);
  const double range = std::max(0.0, measurement(2));
  const double ce = std::cos(elevation);
  TargetState prior;
  prior.position = {range * ce * std::cos(azimuth), range * ce * std::sin(azimuth),
                    range * std::sin(elevation)};
  initialize(prior, pos_var, vel_var, acc_var);
}

Eigen::MatrixXd TargetKalmanFilter::transition(double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n_, n_);
  const double dt2 = dt * dt;
  for (int axis = 0; axis < 3; ++axis) {
    F(axis, axis + 3) = dt;
    if (n_ == 9) {
      F(axis, axis + 6) = 0.5 * dt2;
      F(axis + 3, axis + 6) = dt;
    }
  }
  return F;
}

Eigen::MatrixXd TargetKalmanFilter::processNoise(double dt) const {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n_, n_);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  if (n_ == 6) {
    const double q = accel_std_ * accel_std_;
    for (int axis = 0; axis < 3; ++axis) {
      const int p = axis;
      const int v = axis + 3;
      Q(p, p) = q * dt4 / 4.0;
      Q(p, v) = q * dt3 / 2.0;
      Q(v, p) = Q(p, v);
      Q(v, v) = q * dt2;
    }
    return Q;
  }

  const double q = jerk_std_ * jerk_std_;
  const double g0 = dt3 / 6.0;
  const double g1 = dt2 / 2.0;
  const double g2 = dt;
  const double g[3] = {g0, g1, g2};
  for (int axis = 0; axis < 3; ++axis) {
    const int index[3] = {axis, axis + 3, axis + 6};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        Q(index[row], index[col]) = q * g[row] * g[col];
      }
    }
  }
  return Q;
}

void TargetKalmanFilter::symmetrize() { P_ = 0.5 * (P_ + P_.transpose()); }

void TargetKalmanFilter::predict(double dt) {
  if (!initialized_) {
    throw std::logic_error("Kalman predict called before initialize");
  }
  if (dt <= 0.0) {
    return;
  }
  const Eigen::MatrixXd F = transition(dt);
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + processNoise(dt);
  symmetrize();
}

void TargetKalmanFilter::applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H,
                                     const Eigen::MatrixXd& R) {
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::LLT<Eigen::MatrixXd> llt(S);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("Kalman innovation covariance is not positive definite");
  }
  const int nz = static_cast<int>(innovation.size());
  const Eigen::MatrixXd K = P_ * H.transpose() * llt.solve(Eigen::MatrixXd::Identity(nz, nz));
  x_ += K * innovation;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);
  const Eigen::MatrixXd I_KH = I - K * H;
  // Joseph form keeps P symmetric and positive semidefinite when R is.
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  symmetrize();
  last_nis_ = innovation.dot(llt.solve(innovation));
}

void TargetKalmanFilter::updateCartesian(const Eigen::Vector3d& measurement,
                                         const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("Kalman update called before initialize");
  }
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n_);
  H.block<3, 3>(0, 0).setIdentity();
  const Eigen::Vector3d innovation = measurement - x_.head(3);
  applyUpdate(innovation, H, covariance);
}

void TargetKalmanFilter::bearingRangeModel(Eigen::Vector3d& predicted,
                                           Eigen::MatrixXd& jacobian) const {
  const double px = x_(0);
  const double py = x_(1);
  const double pz = x_(2);
  const double rho_xy = std::hypot(px, py);
  const double rho = std::sqrt(px * px + py * py + pz * pz);
  predicted(0) = std::atan2(py, px);
  predicted(1) = std::atan2(pz, rho_xy);
  predicted(2) = rho;

  jacobian = Eigen::MatrixXd::Zero(3, n_);
  if (rho < 1.0e-9) {
    return;
  }
  const double rho_xy_used = std::max(rho_xy, kRhoXyFloor);
  const double rho_xy2 = rho_xy_used * rho_xy_used;
  const double rho2 = rho * rho;
  if (rho_xy >= kRhoXyFloor) {
    jacobian(0, 0) = -py / rho_xy2;
    jacobian(0, 1) = px / rho_xy2;
    jacobian(1, 0) = -px * pz / (rho2 * rho_xy);
    jacobian(1, 1) = -py * pz / (rho2 * rho_xy);
  }
  jacobian(1, 2) = rho_xy_used / rho2;
  jacobian(2, 0) = px / rho;
  jacobian(2, 1) = py / rho;
  jacobian(2, 2) = pz / rho;
}

void TargetKalmanFilter::updateBearingRange(const Eigen::Vector3d& measurement,
                                            const Eigen::Matrix3d& covariance) {
  if (!initialized_) {
    throw std::logic_error("Kalman update called before initialize");
  }
  Eigen::Vector3d predicted;
  Eigen::MatrixXd H;
  bearingRangeModel(predicted, H);
  Eigen::Vector3d innovation;
  innovation(0) = wrapPi(measurement(0) - predicted(0));
  innovation(1) = measurement(1) - predicted(1);
  innovation(2) = measurement(2) - predicted(2);

  Eigen::Matrix3d R = covariance;
  const double rho_xy = std::hypot(x_(0), x_(1));
  if (rho_xy < kRhoXyFloor) {
    // Azimuth is undefined on the boresight axis. Drop that row.
    H.row(0).setZero();
    innovation(0) = 0.0;
    R.row(0).setZero();
    R.col(0).setZero();
    R(0, 0) = 1.0e6;
  }
  applyUpdate(innovation, H, R);
}

TargetState TargetKalmanFilter::estimate() const {
  TargetState state;
  state.position = {x_(0), x_(1), x_(2)};
  state.velocity = {x_(3), x_(4), x_(5)};
  if (n_ == 9) {
    state.acceleration = {x_(6), x_(7), x_(8)};
  }
  return state;
}

double TargetKalmanFilter::nees(const TargetState& truth) const {
  Eigen::VectorXd truth_vec(n_);
  truth_vec << truth.position.x, truth.position.y, truth.position.z, truth.velocity.x,
      truth.velocity.y, truth.velocity.z;
  if (n_ == 9) {
    truth_vec(6) = truth.acceleration.x;
    truth_vec(7) = truth.acceleration.y;
    truth_vec(8) = truth.acceleration.z;
  }
  const Eigen::VectorXd error = truth_vec - x_;
  const Eigen::MatrixXd P = 0.5 * (P_ + P_.transpose());
  const Eigen::LLT<Eigen::MatrixXd> llt(P);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  return error.dot(llt.solve(error));
}

}  // namespace gimbal_tracker
