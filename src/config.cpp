#include "gimbal_tracker/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace gimbal_tracker {
namespace {

YAML::Node child(const YAML::Node& node, const char* key, const std::string& where) {
  if (!node || !node[key]) {
    throw std::runtime_error("Missing YAML key '" + where + "." + key + "'");
  }
  return node[key];
}

double asDouble(const YAML::Node& node, const char* key, const std::string& where) {
  const YAML::Node value = child(node, key, where);
  try {
    return value.as<double>();
  } catch (const std::exception&) {
    throw std::runtime_error("YAML key '" + where + "." + key + "' must be a number");
  }
}

int asInt(const YAML::Node& node, const char* key, const std::string& where) {
  const YAML::Node value = child(node, key, where);
  try {
    return value.as<int>();
  } catch (const std::exception&) {
    throw std::runtime_error("YAML key '" + where + "." + key + "' must be an integer");
  }
}

bool asBool(const YAML::Node& node, const char* key, const std::string& where) {
  const YAML::Node value = child(node, key, where);
  try {
    return value.as<bool>();
  } catch (const std::exception&) {
    throw std::runtime_error("YAML key '" + where + "." + key + "' must be a bool");
  }
}

std::string asString(const YAML::Node& node, const char* key, const std::string& where) {
  const YAML::Node value = child(node, key, where);
  try {
    return value.as<std::string>();
  } catch (const std::exception&) {
    throw std::runtime_error("YAML key '" + where + "." + key + "' must be a string");
  }
}

double optionalDouble(const YAML::Node& node, const char* key, double fallback) {
  if (!node || !node[key]) {
    return fallback;
  }
  return node[key].as<double>();
}

Vec3 asVec3(const YAML::Node& node, const char* key, const std::string& where) {
  const YAML::Node value = child(node, key, where);
  if (!value.IsSequence() || value.size() != 3) {
    throw std::runtime_error("YAML key '" + where + "." + key + "' must be a sequence of 3 numbers");
  }
  return {value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}

void requirePositive(double value, const std::string& name) {
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::runtime_error(name + " must be positive");
  }
}

void requireNonNegative(double value, const std::string& name) {
  if (value < 0.0 || !std::isfinite(value)) {
    throw std::runtime_error(name + " must be non-negative");
  }
}

AxisPlantConfig loadAxisPlant(const YAML::Node& node, const std::string& where) {
  AxisPlantConfig axis;
  axis.inertia = asDouble(node, "inertia_kg_m2", where);
  axis.viscous = asDouble(node, "viscous_n_m_s", where);
  axis.coulomb = asDouble(node, "coulomb_n_m", where);
  axis.friction_omega = asDouble(node, "friction_smooth_rad_s", where);
  axis.resistance = asDouble(node, "resistance_ohm", where);
  axis.inductance = asDouble(node, "inductance_h", where);
  axis.torque_const = asDouble(node, "torque_const_n_m_per_a", where);
  axis.back_emf = asDouble(node, "back_emf_v_s", where);
  axis.voltage_limit = asDouble(node, "voltage_limit_v", where);
  axis.torque_limit = asDouble(node, "torque_limit_n_m", where);
  axis.rate_limit = asDouble(node, "rate_limit_rad_s", where);
  axis.angle_min = asDouble(node, "angle_min_rad", where);
  axis.angle_max = asDouble(node, "angle_max_rad", where);
  axis.wrap_angle = asBool(node, "wrap_angle", where);
  requirePositive(axis.inertia, where + ".inertia_kg_m2");
  requirePositive(axis.friction_omega, where + ".friction_smooth_rad_s");
  requirePositive(axis.resistance, where + ".resistance_ohm");
  requirePositive(axis.inductance, where + ".inductance_h");
  requirePositive(axis.torque_const, where + ".torque_const_n_m_per_a");
  requirePositive(axis.voltage_limit, where + ".voltage_limit_v");
  requirePositive(axis.torque_limit, where + ".torque_limit_n_m");
  requirePositive(axis.rate_limit, where + ".rate_limit_rad_s");
  requireNonNegative(axis.viscous, where + ".viscous_n_m_s");
  requireNonNegative(axis.coulomb, where + ".coulomb_n_m");
  requireNonNegative(axis.back_emf, where + ".back_emf_v_s");
  if (axis.angle_min >= axis.angle_max) {
    throw std::runtime_error(where + " angle_min_rad must be below angle_max_rad");
  }
  return axis;
}

PidGains loadPid(const YAML::Node& node, const std::string& where) {
  PidGains gains;
  gains.kp = asDouble(node, "kp", where);
  gains.ki = asDouble(node, "ki", where);
  gains.kd = asDouble(node, "kd", where);
  gains.kaw = asDouble(node, "kaw", where);
  requireNonNegative(gains.ki, where + ".ki");
  requireNonNegative(gains.kd, where + ".kd");
  requireNonNegative(gains.kaw, where + ".kaw");
  return gains;
}

AxisControllerConfig loadAxisController(const YAML::Node& node, const std::string& where) {
  AxisControllerConfig axis;
  axis.position = loadPid(child(node, "position", where), where + ".position");
  axis.rate = loadPid(child(node, "rate", where), where + ".rate");
  axis.position_kff = asDouble(node, "position_kff", where);
  axis.rate_kff = asDouble(node, "rate_kff", where);
  axis.rate_kaff = asDouble(node, "rate_kaff", where);
  axis.rate_limit = asDouble(node, "rate_limit_rad_s", where);
  axis.voltage_limit = asDouble(node, "voltage_limit_v", where);
  axis.wrap_error = asBool(node, "wrap_error", where);
  requirePositive(axis.rate_limit, where + ".rate_limit_rad_s");
  requirePositive(axis.voltage_limit, where + ".voltage_limit_v");
  return axis;
}

MotionSegment loadSegment(const YAML::Node& node, const TargetConfig& parent, const std::string& where) {
  MotionSegment segment;
  segment.kind = asString(node, "kind", where);
  segment.duration_s = asDouble(node, "duration_s", where);
  segment.turn_rate = optionalDouble(node, "turn_rate_rad_s", parent.turn_rate);
  segment.speed_m_s = optionalDouble(node, "speed_m_s", parent.speed_final_m_s);
  segment.weave_amplitude = optionalDouble(node, "weave_amplitude_rad", parent.weave_amplitude);
  segment.weave_omega = optionalDouble(node, "weave_omega_rad_s", parent.weave_omega);
  if (segment.kind != "straight" && segment.kind != "coordinated_turn" && segment.kind != "speed_change" &&
      segment.kind != "weave") {
    throw std::runtime_error(where + ".kind must be straight, coordinated_turn, speed_change, or weave");
  }
  requirePositive(segment.duration_s, where + ".duration_s");
  if (segment.kind == "speed_change") {
    requirePositive(segment.speed_m_s, where + ".speed_m_s");
  }
  if (segment.kind == "weave") {
    requirePositive(segment.weave_omega, where + ".weave_omega_rad_s");
    requireNonNegative(std::abs(segment.weave_amplitude), where + ".weave_amplitude_rad");
  }
  return segment;
}

void validateTrajectory(TargetConfig& target, const std::string& where, bool require_segments) {
  const std::string& name = target.trajectory;
  const bool known = name == "constant_velocity" || name == "constant_acceleration" || name == "circle" ||
                     name == "coordinated_turn" || name == "speed_change" || name == "weave" || name == "sequence";
  if (!known) {
    throw std::runtime_error(where +
                             ".trajectory must be constant_velocity, constant_acceleration, circle, "
                             "coordinated_turn, speed_change, weave, or sequence");
  }
  requirePositive(target.circle_radius, where + ".circle_radius_m");
  requireNonNegative(target.speed_ramp_s, where + ".speed_ramp_s");
  requireNonNegative(target.speed_final_m_s, where + ".speed_final_m_s");
  requireNonNegative(target.speed_change_time_s, where + ".speed_change_time_s");
  requireNonNegative(std::abs(target.weave_amplitude), where + ".weave_amplitude_rad");
  if (name == "weave" || name == "sequence") {
    requireNonNegative(std::abs(target.weave_omega), where + ".weave_omega_rad_s");
  }
  if (name == "sequence" && target.segments.empty() && require_segments) {
    throw std::runtime_error(where + " sequence needs at least one segment");
  }
}

TargetConfig loadTarget(const YAML::Node& node, const std::string& where, bool require_segments) {
  TargetConfig target;
  target.position = asVec3(node, "position_m", where);
  target.velocity = asVec3(node, "velocity_m_s", where);
  target.acceleration = asVec3(node, "acceleration_m_s2", where);
  target.trajectory = asString(node, "trajectory", where);
  target.circle_radius = asDouble(node, "circle_radius_m", where);
  target.circle_omega = asDouble(node, "circle_omega_rad_s", where);
  target.circle_z = asDouble(node, "circle_z_m", where);
  target.turn_rate = asDouble(node, "turn_rate_rad_s", where);
  target.weave_amplitude = asDouble(node, "weave_amplitude_rad", where);
  target.weave_omega = asDouble(node, "weave_omega_rad_s", where);
  target.speed_ramp_s = asDouble(node, "speed_ramp_s", where);
  target.speed_final_m_s = asDouble(node, "speed_final_m_s", where);
  target.speed_change_time_s = asDouble(node, "speed_change_time_s", where);
  const YAML::Node segments = child(node, "segments", where);
  if (!segments.IsSequence()) {
    throw std::runtime_error(where + ".segments must be a list");
  }
  for (std::size_t i = 0; i < segments.size(); ++i) {
    target.segments.push_back(loadSegment(segments[i], target, where + ".segments[" + std::to_string(i) + "]"));
  }
  validateTrajectory(target, where, require_segments);
  return target;
}

}  // namespace

SimConfig loadConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& ex) {
    throw std::runtime_error("Failed to read '" + path + "': " + ex.what());
  }

  SimConfig cfg;
  const YAML::Node plant = child(root, "plant", "root");
  cfg.plant.pan = loadAxisPlant(child(plant, "pan", "plant"), "plant.pan");
  cfg.plant.tilt = loadAxisPlant(child(plant, "tilt", "plant"), "plant.tilt");
  cfg.plant.encoder_noise_std = asDouble(plant, "encoder_noise_std_rad", "plant");
  cfg.plant.gyro_noise_std = asDouble(plant, "gyro_noise_std_rad_s", "plant");
  cfg.plant.integration_dt_max = asDouble(plant, "integration_dt_max_s", "plant");
  cfg.plant.seed = static_cast<unsigned>(asInt(plant, "seed", "plant"));
  requireNonNegative(cfg.plant.encoder_noise_std, "plant.encoder_noise_std_rad");
  requireNonNegative(cfg.plant.gyro_noise_std, "plant.gyro_noise_std_rad_s");
  requirePositive(cfg.plant.integration_dt_max, "plant.integration_dt_max_s");

  const YAML::Node controller = child(root, "controller", "root");
  cfg.controller.pan = loadAxisController(child(controller, "pan", "controller"), "controller.pan");
  cfg.controller.tilt = loadAxisController(child(controller, "tilt", "controller"), "controller.tilt");

  const YAML::Node estimator = child(root, "estimator", "root");
  cfg.estimator.model = asString(estimator, "model", "estimator");
  if (cfg.estimator.model != "cv" && cfg.estimator.model != "ca" && cfg.estimator.model != "ct" &&
      cfg.estimator.model != "imm") {
    throw std::runtime_error("estimator.model must be 'cv', 'ca', 'ct', or 'imm'");
  }
  cfg.estimator.accel_std = asDouble(estimator, "accel_std_m_s2", "estimator");
  cfg.estimator.jerk_std = asDouble(estimator, "jerk_std_m_s3", "estimator");
  cfg.estimator.ct_accel_std = asDouble(estimator, "ct_accel_std_m_s2", "estimator");
  cfg.estimator.turn_rate_std = asDouble(estimator, "turn_rate_std_rad_s_sqrt_s", "estimator");
  cfg.estimator.p0_turn_rate = asDouble(estimator, "p0_turn_rate_var", "estimator");
  cfg.estimator.imm_sojourn_cv_s = asDouble(estimator, "imm_sojourn_cv_s", "estimator");
  cfg.estimator.imm_sojourn_ca_s = asDouble(estimator, "imm_sojourn_ca_s", "estimator");
  cfg.estimator.imm_sojourn_ct_s = asDouble(estimator, "imm_sojourn_ct_s", "estimator");
  cfg.estimator.imm_mu_cv = asDouble(estimator, "imm_mu_cv", "estimator");
  cfg.estimator.imm_mu_ca = asDouble(estimator, "imm_mu_ca", "estimator");
  cfg.estimator.imm_mu_ct = asDouble(estimator, "imm_mu_ct", "estimator");
  cfg.estimator.cartesian_std = asDouble(estimator, "cartesian_std_m", "estimator");
  cfg.estimator.azimuth_std = asDouble(estimator, "azimuth_std_rad", "estimator");
  cfg.estimator.elevation_std = asDouble(estimator, "elevation_std_rad", "estimator");
  cfg.estimator.range_std = asDouble(estimator, "range_std_m", "estimator");
  cfg.estimator.p0_pos = asDouble(estimator, "p0_pos_var_m2", "estimator");
  cfg.estimator.p0_vel = asDouble(estimator, "p0_vel_var_m2_s2", "estimator");
  cfg.estimator.p0_acc = asDouble(estimator, "p0_acc_var_m2_s4", "estimator");
  requireNonNegative(cfg.estimator.accel_std, "estimator.accel_std_m_s2");
  requireNonNegative(cfg.estimator.jerk_std, "estimator.jerk_std_m_s3");
  requirePositive(cfg.estimator.cartesian_std, "estimator.cartesian_std_m");
  requirePositive(cfg.estimator.azimuth_std, "estimator.azimuth_std_rad");
  requirePositive(cfg.estimator.elevation_std, "estimator.elevation_std_rad");
  requirePositive(cfg.estimator.range_std, "estimator.range_std_m");
  requirePositive(cfg.estimator.p0_pos, "estimator.p0_pos_var_m2");
  requirePositive(cfg.estimator.p0_vel, "estimator.p0_vel_var_m2_s2");
  requirePositive(cfg.estimator.p0_acc, "estimator.p0_acc_var_m2_s4");
  requireNonNegative(cfg.estimator.ct_accel_std, "estimator.ct_accel_std_m_s2");
  requireNonNegative(cfg.estimator.turn_rate_std, "estimator.turn_rate_std_rad_s_sqrt_s");
  requirePositive(cfg.estimator.p0_turn_rate, "estimator.p0_turn_rate_var");
  requirePositive(cfg.estimator.imm_sojourn_cv_s, "estimator.imm_sojourn_cv_s");
  requirePositive(cfg.estimator.imm_sojourn_ca_s, "estimator.imm_sojourn_ca_s");
  requirePositive(cfg.estimator.imm_sojourn_ct_s, "estimator.imm_sojourn_ct_s");
  if (cfg.estimator.imm_mu_cv < 0.0 || cfg.estimator.imm_mu_ca < 0.0 || cfg.estimator.imm_mu_ct < 0.0) {
    throw std::runtime_error("IMM initial probabilities must be non-negative");
  }

  const YAML::Node lead = child(root, "lead", "root");
  cfg.lead.projectile_speed = asDouble(lead, "projectile_speed_m_s", "lead");
  cfg.lead.gravity = asDouble(lead, "gravity_m_s2", "lead");
  cfg.lead.max_iterations = asInt(lead, "max_iterations", "lead");
  cfg.lead.tolerance = asDouble(lead, "tolerance_s", "lead");
  cfg.lead.muzzle = asVec3(lead, "muzzle_m", "lead");
  requirePositive(cfg.lead.projectile_speed, "lead.projectile_speed_m_s");
  requireNonNegative(cfg.lead.gravity, "lead.gravity_m_s2");
  if (cfg.lead.max_iterations < 1) {
    throw std::runtime_error("lead.max_iterations must be at least 1");
  }
  requirePositive(cfg.lead.tolerance, "lead.tolerance_s");

  cfg.target = loadTarget(child(root, "target", "root"), "target", false);
  cfg.maneuver = loadTarget(child(root, "maneuver", "root"), "maneuver", true);

  const YAML::Node analysis = child(root, "analysis", "root");
  cfg.analysis.step_rad = asDouble(analysis, "step_rad", "analysis");
  cfg.analysis.step_duration_s = asDouble(analysis, "step_duration_s", "analysis");
  cfg.analysis.step_dt_s = asDouble(analysis, "step_dt_s", "analysis");
  cfg.analysis.mc_runs = asInt(analysis, "mc_runs", "analysis");
  cfg.analysis.mc_duration_s = asDouble(analysis, "mc_duration_s", "analysis");
  cfg.analysis.mc_dt_s = asDouble(analysis, "mc_dt_s", "analysis");
  cfg.analysis.mc_seed = static_cast<unsigned>(asInt(analysis, "mc_seed", "analysis"));
  cfg.analysis.mc_pos_var = asDouble(analysis, "mc_pos_var", "analysis");
  cfg.analysis.mc_vel_var = asDouble(analysis, "mc_vel_var", "analysis");
  cfg.analysis.mc_acc_var = asDouble(analysis, "mc_acc_var", "analysis");
  cfg.analysis.mc_accel_std = asDouble(analysis, "mc_accel_std", "analysis");
  cfg.analysis.mc_meas_std = asDouble(analysis, "mc_meas_std", "analysis");
  cfg.analysis.tracking_duration_s = asDouble(analysis, "tracking_duration_s", "analysis");
  cfg.analysis.tracking_dt_s = asDouble(analysis, "tracking_dt_s", "analysis");
  cfg.analysis.imm_runs = asInt(analysis, "imm_runs", "analysis");
  cfg.analysis.imm_duration_s = asDouble(analysis, "imm_duration_s", "analysis");
  cfg.analysis.imm_dt_s = asDouble(analysis, "imm_dt_s", "analysis");
  cfg.analysis.imm_seed = static_cast<unsigned>(asInt(analysis, "imm_seed", "analysis"));
  cfg.analysis.imm_meas_std = asDouble(analysis, "imm_meas_std_m", "analysis");
  requirePositive(cfg.analysis.step_duration_s, "analysis.step_duration_s");
  requirePositive(cfg.analysis.step_dt_s, "analysis.step_dt_s");
  requirePositive(cfg.analysis.mc_duration_s, "analysis.mc_duration_s");
  requirePositive(cfg.analysis.mc_dt_s, "analysis.mc_dt_s");
  requirePositive(cfg.analysis.tracking_duration_s, "analysis.tracking_duration_s");
  requirePositive(cfg.analysis.tracking_dt_s, "analysis.tracking_dt_s");
  requirePositive(cfg.analysis.imm_duration_s, "analysis.imm_duration_s");
  requirePositive(cfg.analysis.imm_dt_s, "analysis.imm_dt_s");
  requirePositive(cfg.analysis.imm_meas_std, "analysis.imm_meas_std_m");
  if (cfg.analysis.mc_runs < 1) {
    throw std::runtime_error("analysis.mc_runs must be at least 1");
  }
  if (cfg.analysis.imm_runs < 1) {
    throw std::runtime_error("analysis.imm_runs must be at least 1");
  }

  const YAML::Node ros = child(root, "ros", "root");
  cfg.ros.rate_hz = asDouble(ros, "rate_hz", "ros");
  cfg.ros.measurement = asString(ros, "measurement", "ros");
  cfg.ros.sensor_seed = static_cast<unsigned>(asInt(ros, "sensor_seed", "ros"));
  cfg.ros.plant_seed = static_cast<unsigned>(asInt(ros, "plant_seed", "ros"));
  requirePositive(cfg.ros.rate_hz, "ros.rate_hz");
  if (cfg.ros.measurement != "cartesian" && cfg.ros.measurement != "bearing_range") {
    throw std::runtime_error("ros.measurement must be 'cartesian' or 'bearing_range'");
  }
  return cfg;
}

}  // namespace gimbal_tracker
