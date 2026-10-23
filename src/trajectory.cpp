#include "gimbal_tracker/trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace gimbal_tracker {

TargetTrajectory::TargetTrajectory(TargetConfig config) : config_(std::move(config)) {}

TargetState TargetTrajectory::at(double time_s) const {
  if (config_.trajectory == "constant_velocity") {
    TargetState state;
    state.position = config_.position + config_.velocity * time_s;
    state.velocity = config_.velocity;
    state.acceleration = Vec3{0.0, 0.0, 0.0};
    return state;
  }
  if (config_.trajectory == "constant_acceleration") {
    TargetState state;
    state.acceleration = config_.acceleration;
    state.velocity = config_.velocity + config_.acceleration * time_s;
    state.position = config_.position + config_.velocity * time_s +
                     config_.acceleration * (0.5 * time_s * time_s);
    return state;
  }
  if (config_.trajectory == "circle") {
    const double angle = config_.circle_omega * time_s;
    const double radius = config_.circle_radius;
    const double omega = config_.circle_omega;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    TargetState state;
    state.position = {radius * c, radius * s, config_.circle_z};
    state.velocity = {-radius * omega * s, radius * omega * c, 0.0};
    state.acceleration = {-radius * omega * omega * c, -radius * omega * omega * s, 0.0};
    return state;
  }
  throw std::invalid_argument("Unknown target trajectory '" + config_.trajectory + "'");
}

}  // namespace gimbal_tracker
