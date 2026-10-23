#include "gimbal_tracker/controller.hpp"

#include "gimbal_tracker/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

double finiteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

}  // namespace

AxisController::AxisController(AxisControllerConfig config)
    : config_(std::move(config)),
      position_pid_(config_.position, -std::abs(config_.rate_limit), std::abs(config_.rate_limit)),
      rate_pid_(config_.rate, -std::abs(config_.voltage_limit), std::abs(config_.voltage_limit)) {}

void AxisController::reset() {
  position_pid_.reset();
  rate_pid_.reset();
  prev_omega_ = 0.0;
  has_prev_omega_ = false;
}

AxisOutput AxisController::update(const AxisInput& input, double dt) {
  if (dt < 0.0) {
    throw std::invalid_argument("AxisController dt must be non-negative");
  }

  double position_error = finiteOrZero(input.theta_ref) - finiteOrZero(input.theta_meas);
  if (config_.wrap_error) {
    position_error = wrapPi(position_error);
  }

  const double position_ff = config_.position_kff * finiteOrZero(input.omega_ff);
  const double rate_command = position_pid_.step(position_error, dt, position_ff, 0.0);

  double omega_derivative = 0.0;
  if (has_prev_omega_ && dt > 0.0) {
    omega_derivative = (input.omega_meas - prev_omega_) / dt;
  }
  prev_omega_ = input.omega_meas;
  has_prev_omega_ = true;

  const double rate_error = rate_command - finiteOrZero(input.omega_meas);
  const double rate_ff =
      config_.rate_kff * rate_command + config_.rate_kaff * finiteOrZero(input.alpha_ff);
  // D on measurement: a step in rate_command does not pass through the derivative.
  const double voltage = rate_pid_.step(rate_error, dt, rate_ff, -omega_derivative);

  AxisOutput output;
  output.voltage = voltage;
  output.rate_command = rate_command;
  output.position_error = position_error;
  return output;
}

GimbalController::GimbalController(ControllerConfig config)
    : pan_(std::move(config.pan)), tilt_(std::move(config.tilt)) {}

void GimbalController::reset() {
  pan_.reset();
  tilt_.reset();
}

ControllerOutput GimbalController::update(const GimbalState& measured, double pan_ref, double tilt_ref,
                                          double dt, double pan_rate_ff, double tilt_rate_ff,
                                          double pan_acc_ff, double tilt_acc_ff) {
  ControllerOutput output;
  output.pan = pan_.update(
      AxisInput{measured.pan.theta, measured.pan.omega, pan_ref, pan_rate_ff, pan_acc_ff}, dt);
  output.tilt = tilt_.update(
      AxisInput{measured.tilt.theta, measured.tilt.omega, tilt_ref, tilt_rate_ff, tilt_acc_ff}, dt);
  output.voltage.pan_voltage = output.pan.voltage;
  output.voltage.tilt_voltage = output.tilt.voltage;
  return output;
}

}  // namespace gimbal_tracker
