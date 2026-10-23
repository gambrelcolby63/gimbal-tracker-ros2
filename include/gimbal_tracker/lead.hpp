#pragma once

#include "gimbal_tracker/types.hpp"

#include <string>

namespace gimbal_tracker {

struct LeadRequest {
  Vec3 muzzle{0.0, 0.0, 0.0};
  Vec3 target_position{0.0, 0.0, 0.0};
  Vec3 target_velocity{0.0, 0.0, 0.0};
  Vec3 target_acceleration{0.0, 0.0, 0.0};
  double projectile_speed{0.0};
  double gravity{0.0};  // m/s^2, positive down-range drop; 0 keeps a straight constant-speed shot
  int max_iterations{12};
  double tolerance{1.0e-6};
};

struct LeadSolution {
  bool valid{false};
  double time_of_flight{0.0};
  Vec3 intercept{};
  double azimuth{0.0};
  double elevation{0.0};
  std::string reason;
};

// Aim angles for a point-mass projectile of constant speed.
// With gravity == 0 and no target acceleration the time of flight is the
// positive root of a quadratic. Gravity uses the classical vacuum elevation
// quadratic inside a fixed-point iteration on the intercept time.
LeadSolution solveLead(const LeadRequest& request);

LookAngles lookAngles(const Vec3& from, const Vec3& to);

}  // namespace gimbal_tracker
