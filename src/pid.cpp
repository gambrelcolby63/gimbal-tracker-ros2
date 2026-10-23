#include "gimbal_tracker/pid.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal_tracker {

Pid::Pid(PidGains gains, double out_min, double out_max)
    : gains_(gains), out_min_(out_min), out_max_(out_max) {
  if (out_min_ > out_max_) {
    std::swap(out_min_, out_max_);
  }
}

void Pid::reset() { integral_ = 0.0; }

double Pid::step(double error, double dt, double feedforward, double error_derivative) {
  const double u_unsat = gains_.kp * error + integral_ + gains_.kd * error_derivative + feedforward;
  const double u = std::clamp(u_unsat, out_min_, out_max_);
  if (dt > 0.0 && std::isfinite(dt)) {
    const double anti_windup = gains_.kaw * (u - u_unsat);
    integral_ += (gains_.ki * error + anti_windup) * dt;
  }
  return u;
}

}  // namespace gimbal_tracker
