#pragma once

#include "gimbal_tracker/config.hpp"
#include "gimbal_tracker/kalman.hpp"

#include <Eigen/Dense>

#include <array>
#include <memory>

namespace gimbal_tracker {

// Horizontal coordinated turn. State is [x, y, z, vx, vy, vz, turn rate].
// z is constant-velocity. A near-zero turn rate falls back to straight flight.
Eigen::Matrix<double, 7, 1> predictCoordinatedTurn(const Eigen::Matrix<double, 7, 1>& state, double dt);
Eigen::Matrix<double, 7, 7> coordinatedTurnJacobian(const Eigen::Matrix<double, 7, 1>& state, double dt);

// Rows are the mode we are leaving, columns the mode we enter.
// Diagonal is exp(-dt / sojourn). The leftover probability is split equally.
Eigen::Matrix3d immTransition(const Eigen::Vector3d& sojourn_s, double dt);

struct GaussianMoment {
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
};

// Modes are ordered CV (6), CA (9), CT (7). Shared position and velocity are
// mixed. Turn rate and acceleration stay with the model that carries them,
// so a straight-flight mode cannot zero the turn rate every cycle.
void mixImmModes(const std::array<GaussianMoment, 3>& modes, const Eigen::Vector3d& probability,
                 const Eigen::Matrix3d& transition, double acceleration_variance,
                 double turn_rate_variance, std::array<GaussianMoment, 3>& mixed,
                 Eigen::Matrix3d& conditional_probability, Eigen::Vector3d& predicted_probability);

// EKF whose horizontal motion is a coordinated turn.
class CoordinatedTurnFilter {
 public:
  CoordinatedTurnFilter(double accel_std, double turn_rate_std);

  void initialize(const TargetState& prior, double pos_var, double vel_var, double turn_rate_var);
  void initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                               double turn_rate_var);
  void initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                                  double turn_rate_var);
  void reset(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
  void predict(double dt);
  void updateCartesian(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);
  void updateBearingRange(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);

  bool initialized() const { return initialized_; }
  const Eigen::VectorXd& state() const { return x_; }
  const Eigen::MatrixXd& covariance() const { return P_; }
  double lastNis() const { return last_nis_; }
  double lastLogLikelihood() const { return last_log_likelihood_; }
  TargetState estimate() const;

 private:
  void applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& R);
  Eigen::Matrix<double, 7, 1> mean7() const;

  double accel_std_;
  double turn_rate_std_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  double last_nis_{0.0};
  double last_log_likelihood_{0.0};
  bool initialized_{false};
};

// Three-model IMM: constant velocity, constant acceleration, coordinated turn.
class InteractingMultipleModel {
 public:
  explicit InteractingMultipleModel(const EstimatorConfig& config);

  void initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                               double acc_var);
  void initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                                  double acc_var);
  void predict(double dt);
  void updateCartesian(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);
  void updateBearingRange(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);

  bool initialized() const { return initialized_; }
  TargetState estimate() const;
  Eigen::Matrix<double, 6, 6> kinematicCovariance() const { return combined_p_; }
  Eigen::Vector3d modelProbability() const { return probability_; }
  double lastNis() const { return last_nis_; }
  double nees(const TargetState& truth) const;

 private:
  void seedFilters(const TargetState& prior, double pos_var, double vel_var, double acc_var);
  void mixAndPredict(double dt);
  void absorbLikelihoods(const Eigen::Vector3d& prior, const Eigen::Vector3d& log_likelihood);
  void combine();
  std::array<GaussianMoment, 3> moments() const;

  EstimatorConfig config_;
  TargetKalmanFilter cv_;
  TargetKalmanFilter ca_;
  CoordinatedTurnFilter ct_;
  Eigen::Vector3d probability_;
  Eigen::Vector3d predicted_probability_;
  Eigen::Matrix<double, 6, 6> combined_p_;
  TargetState combined_;
  double last_nis_{0.0};
  bool initialized_{false};
};

// cv / ca stay on the linear Kalman filter. ct is the turn EKF. imm mixes all three.
class TargetEstimator {
 public:
  explicit TargetEstimator(const EstimatorConfig& config);

  void initializeFromCartesian(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                               double acc_var);
  void initializeFromBearingRange(const Eigen::Vector3d& measurement, double pos_var, double vel_var,
                                  double acc_var);
  void predict(double dt);
  void updateCartesian(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);
  void updateBearingRange(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& covariance);

  bool initialized() const;
  TargetState estimate() const;
  const Eigen::MatrixXd& covariance() const;
  Eigen::Vector3d modelProbability() const;
  double lastNis() const;
  double nees(const TargetState& truth) const;
  const std::string& model() const { return model_; }

 private:
  void refreshView();

  std::string model_;
  double turn_rate_prior_{0.05};
  std::unique_ptr<TargetKalmanFilter> kalman_;
  std::unique_ptr<CoordinatedTurnFilter> turn_;
  std::unique_ptr<InteractingMultipleModel> imm_;
  Eigen::MatrixXd view_p_;
  double view_nis_{0.0};
};

}  // namespace gimbal_tracker
