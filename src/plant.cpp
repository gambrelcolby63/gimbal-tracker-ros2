#include "gimbal_tracker/plant.hpp"

#include "gimbal_tracker/angle.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

double clamp(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }

struct Deriv {
  double theta;
  double omega;
  double current;
};

Deriv derivative(const AxisState& state, double voltage, const AxisPlantConfig& axis) {
  const double applied = clamp(voltage, -axis.voltage_limit, axis.voltage_limit);
  const double friction =
      axis.viscous * state.omega + axis.coulomb * std::tanh(state.omega / axis.friction_omega);
  const double torque =
      clamp(axis.torque_const * state.current - friction, -axis.torque_limit, axis.torque_limit);
  Deriv out;
  out.theta = state.omega;
  out.omega = torque / axis.inertia;
  out.current =
      (applied - axis.resistance * state.current - axis.back_emf * state.omega) / axis.inductance;
  return out;
}

void applyLimits(AxisState& state, const AxisPlantConfig& axis) {
  if (axis.wrap_angle) {
    state.theta = wrapPi(state.theta);
  } else {
    if (state.theta > axis.angle_max) {
      state.theta = axis.angle_max;
      if (state.omega > 0.0) {
        state.omega = 0.0;
      }
    } else if (state.theta < axis.angle_min) {
      state.theta = axis.angle_min;
      if (state.omega < 0.0) {
        state.omega = 0.0;
      }
    }
  }
  state.omega = clamp(state.omega, -axis.rate_limit, axis.rate_limit);
}

void integrateAxis(AxisState& state, double voltage, double dt, const AxisPlantConfig& axis) {
  const Deriv k1 = derivative(state, voltage, axis);

  AxisState s2 = state;
  s2.theta += 0.5 * dt * k1.theta;
  s2.omega += 0.5 * dt * k1.omega;
  s2.current += 0.5 * dt * k1.current;
  const Deriv k2 = derivative(s2, voltage, axis);

  AxisState s3 = state;
  s3.theta += 0.5 * dt * k2.theta;
  s3.omega += 0.5 * dt * k2.omega;
  s3.current += 0.5 * dt * k2.current;
  const Deriv k3 = derivative(s3, voltage, axis);

  AxisState s4 = state;
  s4.theta += dt * k3.theta;
  s4.omega += dt * k3.omega;
  s4.current += dt * k3.current;
  const Deriv k4 = derivative(s4, voltage, axis);

  state.theta += (dt / 6.0) * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
  state.omega += (dt / 6.0) * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega);
  state.current += (dt / 6.0) * (k1.current + 2.0 * k2.current + 2.0 * k3.current + k4.current);
  applyLimits(state, axis);
}

}  // namespace

GimbalPlant::GimbalPlant(PlantConfig config, unsigned seed)
    : config_(std::move(config)), noise_(seed) {}

void GimbalPlant::reset(const GimbalState& state) { truth_ = state; }

GimbalState GimbalPlant::step(const GimbalCommand& command, double dt) {
  if (dt < 0.0 || !std::isfinite(dt)) {
    throw std::invalid_argument("GimbalPlant dt must be finite and non-negative");
  }
  const double h_max = std::max(1.0e-6, config_.integration_dt_max);
  double left = dt;
  while (left > 1.0e-12) {
    const double h = std::min(left, h_max);
    integrateAxis(truth_.pan, command.pan_voltage, h, config_.pan);
    integrateAxis(truth_.tilt, command.tilt_voltage, h, config_.tilt);
    left -= h;
  }
  return truth_;
}

GimbalState GimbalPlant::measure() {
  GimbalState measured = truth_;
  measured.pan.theta += noise_.sample(config_.encoder_noise_std);
  measured.tilt.theta += noise_.sample(config_.encoder_noise_std);
  measured.pan.omega += noise_.sample(config_.gyro_noise_std);
  measured.tilt.omega += noise_.sample(config_.gyro_noise_std);
  if (config_.pan.wrap_angle) {
    measured.pan.theta = wrapPi(measured.pan.theta);
  }
  measured.pan.current = 0.0;
  measured.tilt.current = 0.0;
  return measured;
}

}  // namespace gimbal_tracker
