#include "gimbal_tracker/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gimbal_tracker {
namespace {

constexpr int kBesselTerms = 8;

struct Flat {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
};

double horizontalSpeed(const Vec3& velocity) { return std::hypot(velocity.x, velocity.y); }

void weaveIntegrals(double amplitude, double omega, double tau, double& integral_cos, double& integral_sin) {
  if (!(std::abs(omega) > 1.0e-8) || !(std::abs(tau) > 0.0)) {
    integral_cos = tau;
    integral_sin = 0.0;
    return;
  }
  integral_cos = std::cyl_bessel_j(0, amplitude) * tau;
  integral_sin = 0.0;
  for (int k = 1; k <= kBesselTerms; ++k) {
    const double even = static_cast<double>(2 * k);
    const double odd = even - 1.0;
    integral_cos += 2.0 * std::cyl_bessel_j(2 * k, amplitude) * std::sin(even * omega * tau) / (even * omega);
    integral_sin +=
        2.0 * std::cyl_bessel_j(2 * k - 1, amplitude) * (1.0 - std::cos(odd * omega * tau)) / (odd * omega);
  }
}

Flat fromConfig(const TargetConfig& config) {
  Flat state;
  state.position = config.position;
  state.velocity = config.velocity;
  state.acceleration = {0.0, 0.0, 0.0};
  return state;
}

Flat advanceStraight(Flat state, double tau) {
  state.position = state.position + state.velocity * tau;
  state.acceleration = {0.0, 0.0, 0.0};
  return state;
}

Flat advanceTurn(Flat state, double omega, double tau) {
  const double vx = state.velocity.x;
  const double vy = state.velocity.y;
  const double vz = state.velocity.z;
  if (std::abs(omega) < 1.0e-8) {
    return advanceStraight(state, tau);
  }
  const double s = std::sin(omega * tau);
  const double c = std::cos(omega * tau);
  state.position.x += (vx * s - vy * (1.0 - c)) / omega;
  state.position.y += (vx * (1.0 - c) + vy * s) / omega;
  state.position.z += vz * tau;
  state.velocity.x = vx * c - vy * s;
  state.velocity.y = vx * s + vy * c;
  state.velocity.z = vz;
  state.acceleration = {-omega * state.velocity.y, omega * state.velocity.x, 0.0};
  return state;
}

Flat advanceSpeed(Flat state, double speed_final, double ramp_s, double tau) {
  const double speed0 = std::max(horizontalSpeed(state.velocity), 0.0);
  const double heading = std::atan2(state.velocity.y, state.velocity.x);
  const double ramp = std::max(ramp_s, 1.0e-6);
  const double accel = (speed_final - speed0) / ramp;
  double distance = 0.0;
  double speed = speed0;
  double accel_now = 0.0;
  if (tau <= ramp) {
    distance = speed0 * tau + 0.5 * accel * tau * tau;
    speed = speed0 + accel * tau;
    accel_now = accel;
  } else {
    distance = speed0 * ramp + 0.5 * accel * ramp * ramp + speed_final * (tau - ramp);
    speed = speed_final;
  }
  const double ch = std::cos(heading);
  const double sh = std::sin(heading);
  state.position.x += distance * ch;
  state.position.y += distance * sh;
  state.position.z += state.velocity.z * tau;
  state.velocity.x = speed * ch;
  state.velocity.y = speed * sh;
  state.acceleration = {accel_now * ch, accel_now * sh, 0.0};
  return state;
}

Flat advanceWeave(Flat state, double amplitude, double omega, double tau) {
  const double speed = horizontalSpeed(state.velocity);
  const double heading0 = std::atan2(state.velocity.y, state.velocity.x);
  double integral_cos = 0.0;
  double integral_sin = 0.0;
  weaveIntegrals(amplitude, omega, tau, integral_cos, integral_sin);
  const double ch = std::cos(heading0);
  const double sh = std::sin(heading0);
  state.position.x += speed * (ch * integral_cos - sh * integral_sin);
  state.position.y += speed * (sh * integral_cos + ch * integral_sin);
  state.position.z += state.velocity.z * tau;
  const double psi = heading0 + amplitude * std::sin(omega * tau);
  state.velocity.x = speed * std::cos(psi);
  state.velocity.y = speed * std::sin(psi);
  const double turn = amplitude * omega * std::cos(omega * tau);
  state.acceleration = {-turn * state.velocity.y, turn * state.velocity.x, 0.0};
  return state;
}

Flat advanceSegment(Flat state, const MotionSegment& segment, double tau, double ramp_s) {
  const double used = std::clamp(tau, 0.0, std::max(segment.duration_s, 0.0));
  if (segment.kind == "straight") {
    return advanceStraight(state, used);
  }
  if (segment.kind == "coordinated_turn") {
    return advanceTurn(state, segment.turn_rate, used);
  }
  if (segment.kind == "speed_change") {
    return advanceSpeed(state, segment.speed_m_s, std::min(ramp_s, segment.duration_s), used);
  }
  if (segment.kind == "weave") {
    return advanceWeave(state, segment.weave_amplitude, segment.weave_omega, used);
  }
  throw std::invalid_argument("Unknown maneuver segment '" + segment.kind + "'");
}

TargetState asTarget(const Flat& state) {
  TargetState out;
  out.position = state.position;
  out.velocity = state.velocity;
  out.acceleration = state.acceleration;
  return out;
}

}  // namespace

TargetTrajectory::TargetTrajectory(TargetConfig config) : config_(std::move(config)) {}

double TargetTrajectory::duration() const {
  if (config_.trajectory != "sequence") {
    return std::numeric_limits<double>::infinity();
  }
  double total = 0.0;
  for (const MotionSegment& segment : config_.segments) {
    total += segment.duration_s;
  }
  return total;
}

std::string TargetTrajectory::kindAt(double time_s) const {
  if (config_.trajectory != "sequence") {
    if (config_.trajectory == "speed_change" && time_s >= config_.speed_change_time_s &&
        time_s < config_.speed_change_time_s + config_.speed_ramp_s) {
      return "speed_change";
    }
    if (config_.trajectory == "speed_change" && time_s >= config_.speed_change_time_s + config_.speed_ramp_s) {
      return "straight";
    }
    if (config_.trajectory == "speed_change") {
      return "straight";
    }
    return config_.trajectory;
  }
  double cursor = 0.0;
  for (const MotionSegment& segment : config_.segments) {
    if (time_s < cursor + segment.duration_s) {
      return segment.kind;
    }
    cursor += segment.duration_s;
  }
  return config_.segments.empty() ? "straight" : config_.segments.back().kind;
}

TargetState TargetTrajectory::at(double time_s) const {
  const double time = std::max(0.0, time_s);
  if (config_.trajectory == "constant_velocity") {
    TargetState state;
    state.position = config_.position + config_.velocity * time;
    state.velocity = config_.velocity;
    state.acceleration = Vec3{0.0, 0.0, 0.0};
    return state;
  }
  if (config_.trajectory == "constant_acceleration") {
    TargetState state;
    state.acceleration = config_.acceleration;
    state.velocity = config_.velocity + config_.acceleration * time;
    state.position = config_.position + config_.velocity * time + config_.acceleration * (0.5 * time * time);
    return state;
  }
  if (config_.trajectory == "circle") {
    const double angle = config_.circle_omega * time;
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
  if (config_.trajectory == "coordinated_turn") {
    return asTarget(advanceTurn(fromConfig(config_), config_.turn_rate, time));
  }
  if (config_.trajectory == "weave") {
    return asTarget(advanceWeave(fromConfig(config_), config_.weave_amplitude, config_.weave_omega, time));
  }
  if (config_.trajectory == "speed_change") {
    Flat state = fromConfig(config_);
    const double coast = std::min(time, std::max(0.0, config_.speed_change_time_s));
    state = advanceStraight(state, coast);
    if (time > config_.speed_change_time_s) {
      state = advanceSpeed(state, config_.speed_final_m_s, config_.speed_ramp_s, time - config_.speed_change_time_s);
    }
    return asTarget(state);
  }
  if (config_.trajectory == "sequence") {
    if (config_.segments.empty()) {
      throw std::invalid_argument("sequence trajectory needs at least one segment");
    }
    Flat state = fromConfig(config_);
    double remaining = time;
    for (const MotionSegment& segment : config_.segments) {
      const double step = std::min(remaining, std::max(0.0, segment.duration_s));
      state = advanceSegment(state, segment, step, config_.speed_ramp_s);
      remaining -= step;
      if (!(remaining > 0.0)) {
        return asTarget(state);
      }
    }
    return asTarget(advanceStraight(state, remaining));
  }
  throw std::invalid_argument("Unknown target trajectory '" + config_.trajectory + "'");
}

}  // namespace gimbal_tracker
