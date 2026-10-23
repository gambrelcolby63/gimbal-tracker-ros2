#pragma once

#include "gimbal_tracker/config.hpp"

namespace gimbal_tracker {

// Deterministic target motion used by the simulator and the ROS target node.
// constant_velocity ignores acceleration. constant_acceleration uses the
// configured acceleration. circle is a horizontal coordinated turn about the
// origin, which a CV model cannot represent exactly.
class TargetTrajectory {
 public:
  explicit TargetTrajectory(TargetConfig config);

  TargetState at(double time_s) const;
  const TargetConfig& config() const { return config_; }

 private:
  TargetConfig config_;
};

}  // namespace gimbal_tracker
