#pragma once

#include "gimbal_tracker/angle.hpp"

namespace gimbal_tracker {

struct AxisState {
  double theta{0.0};    // rad
  double omega{0.0};    // rad/s
  double current{0.0};  // A
};

struct GimbalState {
  AxisState pan;
  AxisState tilt;
};

struct GimbalCommand {
  double pan_voltage{0.0};
  double tilt_voltage{0.0};
};

struct TargetState {
  Vec3 position;      // m, turret frame
  Vec3 velocity;      // m/s
  Vec3 acceleration;  // m/s^2
};

struct LookAngles {
  double azimuth{0.0};    // rad
  double elevation{0.0};  // rad
  double range{0.0};      // m
};

}  // namespace gimbal_tracker
