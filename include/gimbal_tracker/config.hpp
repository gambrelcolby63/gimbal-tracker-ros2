#pragma once

#include "gimbal_tracker/types.hpp"

#include <string>
#include <vector>

namespace gimbal_tracker {

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double kaw{0.0};  // back-calculation gain, 1/s
};

struct AxisPlantConfig {
  double inertia{0.05};
  double viscous{0.02};
  double coulomb{0.0};
  double friction_omega{0.05};
  double resistance{2.0};
  double inductance{0.002};
  double torque_const{0.1};
  double back_emf{0.1};
  double voltage_limit{24.0};
  double torque_limit{2.0};
  double rate_limit{8.0};
  double angle_min{-kPi};
  double angle_max{kPi};
  bool wrap_angle{false};
};

struct PlantConfig {
  AxisPlantConfig pan;
  AxisPlantConfig tilt;
  double encoder_noise_std{0.0};
  double gyro_noise_std{0.0};
  double integration_dt_max{5.0e-4};
  unsigned seed{1};
};

struct AxisControllerConfig {
  PidGains position;
  PidGains rate;
  double position_kff{1.0};  // gain on the external rate feedforward, rad/s per rad/s
  double rate_kff{0.0};      // V per (rad/s) of rate command
  double rate_kaff{0.0};     // V per (rad/s^2) of acceleration feedforward
  double rate_limit{2.0};
  double voltage_limit{24.0};
  bool wrap_error{false};
};

struct ControllerConfig {
  AxisControllerConfig pan;
  AxisControllerConfig tilt;
};

struct EstimatorConfig {
  std::string model{"cv"};  // "cv", "ca", "ct", or "imm"
  double accel_std{0.5};    // piecewise-constant acceleration std, CV process model
  double jerk_std{1.0};     // piecewise-constant jerk std, CA process model
  double ct_accel_std{0.5}; // small along-track noise inside the turn model
  double turn_rate_std{0.2};  // random-walk intensity of turn rate, rad/s/sqrt(s)
  double cartesian_std{0.4};
  double azimuth_std{0.002};
  double elevation_std{0.002};
  double range_std{0.5};
  double p0_pos{25.0};
  double p0_vel{100.0};
  double p0_acc{25.0};
  double p0_turn_rate{0.05};  // rad^2/s^2
  // Mean time a mode stays selected before the Markov chain leaves it.
  double imm_sojourn_cv_s{8.0};
  double imm_sojourn_ca_s{4.0};
  double imm_sojourn_ct_s{5.0};
  double imm_mu_cv{0.5};
  double imm_mu_ca{0.25};
  double imm_mu_ct{0.25};
};

struct LeadConfig {
  double projectile_speed{220.0};
  double gravity{0.0};
  int max_iterations{12};
  double tolerance{1.0e-6};
  Vec3 muzzle{0.0, 0.0, 0.0};
};

// One leg of a maneuver sequence. Unused fields stay at 0 and the parent
// TargetConfig supplies the default turn rate or weave.
struct MotionSegment {
  std::string kind{"straight"};  // straight, coordinated_turn, speed_change, weave
  double duration_s{1.0};
  double turn_rate{0.0};         // rad/s, positive is a left turn
  double speed_m_s{0.0};         // horizontal speed at the end of a speed_change
  double weave_amplitude{0.0};   // rad of heading
  double weave_omega{0.0};       // rad/s
};

struct TargetConfig {
  Vec3 position{50.0, -40.0, 3.0};
  Vec3 velocity{0.0, 12.0, 0.0};
  Vec3 acceleration{0.0, 0.0, 0.0};
  // constant_velocity, constant_acceleration, circle, coordinated_turn,
  // speed_change, weave, or sequence.
  std::string trajectory{"constant_velocity"};
  double circle_radius{30.0};
  double circle_omega{0.2};
  double circle_z{5.0};
  double turn_rate{0.3};
  double weave_amplitude{0.4};
  double weave_omega{1.0};
  double speed_ramp_s{1.0};
  double speed_final_m_s{20.0};
  double speed_change_time_s{4.0};
  std::vector<MotionSegment> segments;
};

struct AnalysisConfig {
  double step_rad{0.25};
  double step_duration_s{4.0};
  double step_dt_s{0.001};
  int mc_runs{50};
  double mc_duration_s{5.0};
  double mc_dt_s{0.05};
  unsigned mc_seed{7};
  double mc_pos_var{4.0};
  double mc_vel_var{4.0};
  double mc_acc_var{4.0};
  double mc_accel_std{1.0};
  double mc_meas_std{0.5};
  double tracking_duration_s{8.0};
  double tracking_dt_s{0.01};
  int imm_runs{40};
  double imm_duration_s{20.0};
  double imm_dt_s{0.05};
  unsigned imm_seed{21};
  double imm_meas_std{1.0};
};

struct RosConfig {
  double rate_hz{200.0};
  std::string measurement{"cartesian"};  // cartesian or bearing_range
  unsigned sensor_seed{11};
  unsigned plant_seed{13};
};

struct SimConfig {
  PlantConfig plant;
  ControllerConfig controller;
  EstimatorConfig estimator;
  LeadConfig lead;
  TargetConfig target;
  TargetConfig maneuver;  // scenario used by the IMM comparison, not the flyby
  AnalysisConfig analysis;
  RosConfig ros;
};

SimConfig loadConfig(const std::string& path);

}  // namespace gimbal_tracker
