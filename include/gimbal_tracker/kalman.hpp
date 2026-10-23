#pragma once

#include "gimbal_tracker/types.hpp"

#include <Eigen/Dense>
#include <string>

namespace gimbal_tracker {

// Linear Kalman filter for a constant-velocity or constant-acceleration target,
// plus a first-order EKF update for bearing/range measurements.
//
// CV state is [x, y, z, vx, vy, vz]. CA appends [ax, ay, az].
// Process noise is piecewise-constant white acceleration (CV) or jerk (CA),
// independent per axis. See the README for F and Q.
class TargetKalmanFilter {
 public:
  enum class Model { ConstantVelocity, ConstantAcceleration };

  TargetKalmanFilter(Model model, double accel_std, double jerk_std);

  static Model modelFromString(const std::string& name);

  void initialize(const TargetState& prior_mean, double pos_var, double vel_var, double acc_var);
  void initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                               double acc_var);
  void initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var,
                                  double vel_var, double acc_var);
  // Replace the posterior. Used by the IMM to inject a mixed initial condition.
  void reset(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);

  void predict(double dt);
  void updateCartesian(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);
  // measurement is [azimuth, elevation, range]. Azimuth innovation is wrapped.
  void updateBearingRange(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);

  bool initialized() const { return initialized_; }
  Model model() const { return model_; }
  int dimension() const { return n_; }
  TargetState estimate() const;
  const Eigen::VectorXd& state() const { return x_; }
  const Eigen::MatrixXd& covariance() const { return P_; }
  double lastNis() const { return last_nis_; }
  // Gaussian log density of the latest innovation, including log|S| and 2π.
  double lastLogLikelihood() const { return last_log_likelihood_; }

  // Normalized estimation error squared against the matching truth components.
  double nees(const TargetState& truth) const;

  // Predicted measurement and Jacobian at the current state. Used by tests.
  void bearingRangeModel(Eigen::Vector3d& predicted, Eigen::MatrixXd& jacobian) const;

 private:
  void applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& R);
  Eigen::MatrixXd transition(double dt) const;
  Eigen::MatrixXd processNoise(double dt) const;
  void symmetrize();

  Model model_;
  int n_;
  double accel_std_;
  double jerk_std_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  double last_nis_{0.0};
  double last_log_likelihood_{0.0};
  bool initialized_{false};
};

}  // namespace gimbal_tracker
