#include "gimbal_tracker/gimbal_tracker.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace {

int fail(const std::string& message) {
  std::cerr << message << "\n";
  return 1;
}

gimbal_tracker::LeadRequest makeLead(const gimbal_tracker::SimConfig& config,
                                     const gimbal_tracker::TargetState& estimate) {
  gimbal_tracker::LeadRequest request;
  request.muzzle = config.lead.muzzle;
  request.target_position = estimate.position;
  request.target_velocity = estimate.velocity;
  request.target_acceleration = estimate.acceleration;
  request.projectile_speed = config.lead.projectile_speed;
  request.gravity = config.lead.gravity;
  request.max_iterations = config.lead.max_iterations;
  request.tolerance = config.lead.tolerance;
  return request;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: tracking_demo <config.yaml> <output.csv>\n";
    return 2;
  }
  gimbal_tracker::SimConfig config;
  try {
    config = gimbal_tracker::loadConfig(argv[1]);
  } catch (const std::exception& ex) {
    return fail(ex.what());
  }

  std::ofstream out(argv[2]);
  if (!out) {
    return fail(std::string("Could not write ") + argv[2]);
  }
  out << "t,los_az,los_el,cmd_az,cmd_el,gimbal_az,gimbal_el,pos_err,nees,tof,valid,"
         "truth_x,truth_y,truth_z,meas_x,meas_y,meas_z,est_x,est_y,est_z,"
         "aim_x,aim_y,aim_z,pan_err,tilt_err\n";
  out.precision(16);

  const gimbal_tracker::TargetTrajectory trajectory(config.target);
  gimbal_tracker::GimbalPlant plant(config.plant, config.plant.seed);
  gimbal_tracker::GimbalController controller(config.controller);
  gimbal_tracker::TargetKalmanFilter filter(
      gimbal_tracker::TargetKalmanFilter::modelFromString(config.estimator.model),
      config.estimator.accel_std, config.estimator.jerk_std);
  gimbal_tracker::GaussianNoise sensor(config.ros.sensor_seed);

  const double dt = config.analysis.tracking_dt_s;
  const int steps = static_cast<int>(std::lround(config.analysis.tracking_duration_s / dt));
  const bool bearing = config.ros.measurement == "bearing_range";
  double pan_sq = 0.0;
  double tilt_sq = 0.0;

  const auto cartesian_from_bearing = [&](const Eigen::Vector3d& z) {
    const double azimuth = z(0);
    const double elevation = z(1);
    const double range = z(2);
    const double ce = std::cos(elevation);
    Eigen::Vector3d position;
    position << config.lead.muzzle.x + range * ce * std::cos(azimuth),
        config.lead.muzzle.y + range * ce * std::sin(azimuth),
        config.lead.muzzle.z + range * std::sin(elevation);
    return position;
  };

  for (int k = 0; k <= steps; ++k) {
    const double time = k * dt;
    const gimbal_tracker::TargetState truth = trajectory.at(time);
    const gimbal_tracker::LookAngles los =
        gimbal_tracker::lookAngles(config.lead.muzzle, truth.position);

    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    if (!filter.initialized()) {
      if (bearing) {
        Eigen::Vector3d z;
        z << gimbal_tracker::wrapPi(los.azimuth + sensor.sample(config.estimator.azimuth_std)),
            los.elevation + sensor.sample(config.estimator.elevation_std),
            std::max(0.05, los.range + sensor.sample(config.estimator.range_std));
        filter.initializeFromBearingRange(z, config.estimator.p0_pos, config.estimator.p0_vel,
                                          config.estimator.p0_acc);
        measurement = cartesian_from_bearing(z);
      } else {
        measurement << truth.position.x + sensor.sample(config.estimator.cartesian_std),
            truth.position.y + sensor.sample(config.estimator.cartesian_std),
            truth.position.z + sensor.sample(config.estimator.cartesian_std);
        filter.initializeFromCartesian(measurement, config.estimator.p0_pos, config.estimator.p0_vel,
                                       config.estimator.p0_acc);
      }
    } else {
      filter.predict(dt);
      if (bearing) {
        Eigen::Vector3d z;
        z << gimbal_tracker::wrapPi(los.azimuth + sensor.sample(config.estimator.azimuth_std)),
            los.elevation + sensor.sample(config.estimator.elevation_std),
            std::max(0.05, los.range + sensor.sample(config.estimator.range_std));
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        covariance(0, 0) = config.estimator.azimuth_std * config.estimator.azimuth_std;
        covariance(1, 1) = config.estimator.elevation_std * config.estimator.elevation_std;
        covariance(2, 2) = config.estimator.range_std * config.estimator.range_std;
        filter.updateBearingRange(z, covariance);
        measurement = cartesian_from_bearing(z);
      } else {
        measurement << truth.position.x + sensor.sample(config.estimator.cartesian_std),
            truth.position.y + sensor.sample(config.estimator.cartesian_std),
            truth.position.z + sensor.sample(config.estimator.cartesian_std);
        const double variance = config.estimator.cartesian_std * config.estimator.cartesian_std;
        filter.updateCartesian(measurement, Eigen::Matrix3d::Identity() * variance);
      }
    }

    const gimbal_tracker::TargetState estimate = filter.estimate();
    const gimbal_tracker::LeadSolution lead = gimbal_tracker::solveLead(makeLead(config, estimate));
    double command_az = los.azimuth;
    double command_el = los.elevation;
    if (lead.valid) {
      command_az = lead.azimuth;
      command_el = lead.elevation;
    } else {
      const gimbal_tracker::LookAngles estimated_los =
          gimbal_tracker::lookAngles(config.lead.muzzle, estimate.position);
      command_az = estimated_los.azimuth;
      command_el = estimated_los.elevation;
    }

    const gimbal_tracker::GimbalState measured = plant.measure();
    const auto command = controller.update(measured, command_az, command_el, dt);
    const double gimbal_az = plant.truth().pan.theta;
    const double gimbal_el = plant.truth().tilt.theta;
    const double pos_err = (estimate.position - truth.position).norm();
    const double pan_err = gimbal_tracker::wrapPi(command_az - gimbal_az);
    const double tilt_err = command_el - gimbal_el;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double aim_x = lead.valid ? lead.intercept.x : nan;
    const double aim_y = lead.valid ? lead.intercept.y : nan;
    const double aim_z = lead.valid ? lead.intercept.z : nan;
    out << time << "," << los.azimuth << "," << los.elevation << "," << command_az << "," << command_el
        << "," << gimbal_az << "," << gimbal_el << "," << pos_err << "," << filter.nees(truth) << ","
        << lead.time_of_flight << "," << (lead.valid ? 1 : 0) << "," << truth.position.x << ","
        << truth.position.y << "," << truth.position.z << "," << measurement(0) << "," << measurement(1)
        << "," << measurement(2) << "," << estimate.position.x << "," << estimate.position.y << ","
        << estimate.position.z << "," << aim_x << "," << aim_y << "," << aim_z << "," << pan_err << ","
        << tilt_err << "\n";

    if (time > 1.0) {
      const double pan_error = gimbal_tracker::wrapPi(command_az - gimbal_az);
      pan_sq += pan_error * pan_error;
      const double tilt_error = command_el - gimbal_el;
      tilt_sq += tilt_error * tilt_error;
    }
    if (k == steps) {
      break;
    }
    plant.step(command.voltage, dt);
  }

  const double samples = std::max(1.0, config.analysis.tracking_duration_s / dt);
  std::cout << "Wrote " << argv[2] << "  tracking RMS pan=" << std::sqrt(pan_sq / samples)
            << " rad  tilt=" << std::sqrt(tilt_sq / samples) << " rad\n";
  return 0;
}
