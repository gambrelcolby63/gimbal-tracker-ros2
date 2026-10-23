#pragma once

#include "gimbal_tracker/config.hpp"

namespace gimbal_tracker {

// Deterministic target motion used by the simulator and the ROS target node.
// constant_velocity ignores acceleration. constant_acceleration uses the
// configured acceleration. circle is a horizontal orbit about the origin.
// coordinated_turn, speed_change, and weave are single maneuvers. sequence
// chains straight, turn, speed-change, and weave legs from the YAML list.
class TargetTrajectory {
 public:
  explicit TargetTrajectory(TargetConfig config);

  TargetState at(double time_s) const;
  // Active leg, or the trajectory name when there is no segment list.
  std::string kindAt(double time_s) const;
  double duration() const;
  const TargetConfig& config() const { return config_; }

 private:
  TargetConfig config_;
};

}  // namespace gimbal_tracker
