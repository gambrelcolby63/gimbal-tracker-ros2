#pragma once

#include "gimbal_tracker/config.hpp"
#include "gimbal_tracker/pid.hpp"
#include "gimbal_tracker/types.hpp"

namespace gimbal_tracker {

struct AxisInput {
  double theta_meas{0.0};
  double omega_meas{0.0};
  double theta_ref{0.0};
  double omega_ff{0.0};
  double alpha_ff{0.0};
};

struct AxisOutput {
  double voltage{0.0};
  double rate_command{0.0};
  double position_error{0.0};
};

struct ControllerOutput {
  GimbalCommand voltage;
  AxisOutput pan;
  AxisOutput tilt;
};

// One cascaded position/rate axis.
//
//   rate_cmd = sat(Kp_p e_theta + integral_p + Kff_p omega_ff)
//   voltage  = sat(Kp_r e_rate + integral_r + Kff_r rate_cmd + Kaff alpha_ff)
//
// When wrap_error is set, e_theta is wrapped to (-pi, pi]. Derivative action,
// if kd is nonzero, uses the measured rate so a step in the command does not
// kick the D term.
class AxisController {
 public:
  explicit AxisController(AxisControllerConfig config);

  void reset();
  AxisOutput update(const AxisInput& input, double dt);
  double rateIntegral() const { return rate_pid_.integral(); }
  double positionIntegral() const { return position_pid_.integral(); }
  const AxisControllerConfig& config() const { return config_; }

 private:
  AxisControllerConfig config_;
  Pid position_pid_;
  Pid rate_pid_;
  double prev_omega_{0.0};
  bool has_prev_omega_{false};
};

class GimbalController {
 public:
  explicit GimbalController(ControllerConfig config);

  void reset();
  ControllerOutput update(const GimbalState& measured, double pan_ref, double tilt_ref, double dt,
                          double pan_rate_ff = 0.0, double tilt_rate_ff = 0.0,
                          double pan_acc_ff = 0.0, double tilt_acc_ff = 0.0);

  double panRateIntegral() const { return pan_.rateIntegral(); }
  double tiltRateIntegral() const { return tilt_.rateIntegral(); }

 private:
  AxisController pan_;
  AxisController tilt_;
};

}  // namespace gimbal_tracker
