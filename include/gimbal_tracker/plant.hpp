#pragma once

#include "gimbal_tracker/config.hpp"
#include "gimbal_tracker/noise.hpp"

namespace gimbal_tracker {

// Two-axis gimbal. Each axis is an independent DC motor on a rigid inertia:
//
//   L di/dt = v - R i - Kb omega
//   J domega/dt = sat(Kt i - b omega - tau_c tanh(omega / omega_s))
//   dtheta/dt = omega
//
// Voltage, torque, and rate are saturated. A non-wrapping axis also stops at
// its angle limits. measure() adds encoder and gyro noise; the integrated
// state returned by step() stays noise-free.
class GimbalPlant {
 public:
  GimbalPlant(PlantConfig config, unsigned seed);

  void reset(const GimbalState& state);
  GimbalState step(const GimbalCommand& command, double dt);
  GimbalState measure();

  const GimbalState& truth() const { return truth_; }
  const PlantConfig& config() const { return config_; }

 private:
  PlantConfig config_;
  GimbalState truth_;
  GaussianNoise noise_;
};

}  // namespace gimbal_tracker
