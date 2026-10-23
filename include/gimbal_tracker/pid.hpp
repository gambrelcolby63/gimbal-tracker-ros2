#pragma once

#include "gimbal_tracker/config.hpp"

namespace gimbal_tracker {

// PID with back-calculation anti-windup. The stored integral is the integrator
// output (it already includes ki), so a saturated actuator stops the integral
// from running away:
//
//   u_unsat = kp e + integral + kd e_dot + feedforward
//   u = sat(u_unsat)
//   integral += (ki e + kaw (u - u_unsat)) dt
class Pid {
 public:
  Pid() = default;
  Pid(PidGains gains, double out_min, double out_max);

  void reset();
  double integral() const { return integral_; }
  const PidGains& gains() const { return gains_; }

  double step(double error, double dt, double feedforward, double error_derivative);

 private:
  PidGains gains_{};
  double out_min_{-1.0e300};
  double out_max_{1.0e300};
  double integral_{0.0};
};

}  // namespace gimbal_tracker
