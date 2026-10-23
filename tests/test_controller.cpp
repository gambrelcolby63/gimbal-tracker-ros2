#include "gimbal_tracker/gimbal_tracker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

using namespace gimbal_tracker;

AxisPlantConfig testAxis() {
  AxisPlantConfig axis;
  axis.inertia = 0.05;
  axis.viscous = 0.02;
  axis.coulomb = 0.005;
  axis.friction_omega = 0.05;
  axis.resistance = 2.0;
  axis.inductance = 0.002;
  axis.torque_const = 0.1;
  axis.back_emf = 0.1;
  axis.voltage_limit = 24.0;
  axis.torque_limit = 2.0;
  axis.rate_limit = 8.0;
  axis.angle_min = -1.4;
  axis.angle_max = 1.4;
  axis.wrap_angle = false;
  return axis;
}

PlantConfig testPlantConfig() {
  PlantConfig plant;
  plant.pan = testAxis();
  plant.tilt = testAxis();
  plant.encoder_noise_std = 0.0;
  plant.gyro_noise_std = 0.0;
  plant.integration_dt_max = 5.0e-4;
  plant.seed = 1;
  return plant;
}

AxisControllerConfig testAxisController() {
  AxisControllerConfig axis;
  axis.position.kp = 4.0;
  axis.position.ki = 0.0;
  axis.position.kd = 0.0;
  axis.position.kaw = 4.0;
  axis.rate.kp = 6.0;
  axis.rate.ki = 40.0;
  axis.rate.kd = 0.0;
  axis.rate.kaw = 12.0;
  axis.position_kff = 1.0;
  axis.rate_kff = 0.5;
  axis.rate_kaff = 1.0;
  axis.rate_limit = 3.0;
  axis.voltage_limit = 24.0;
  axis.wrap_error = true;
  return axis;
}

ControllerConfig testControllerConfig() {
  ControllerConfig config;
  config.pan = testAxisController();
  config.tilt = testAxisController();
  config.tilt.wrap_error = false;
  return config;
}

struct StepStats {
  double rise_time{0.0};
  double overshoot_percent{0.0};
  double settling_time{0.0};
  double steady_state_error{0.0};
  bool reached_90{false};
  bool stayed_settled{false};
};

StepStats runPanStep(const PlantConfig& plant_config, const ControllerConfig& controller_config,
                     double step, double duration, double dt) {
  GimbalPlant plant(plant_config, 1);
  GimbalController controller(controller_config);
  const int steps = static_cast<int>(std::lround(duration / dt));
  double t10 = -1.0;
  double t90 = -1.0;
  double peak = 0.0;
  double last_outside = 0.0;
  double ess_sum = 0.0;
  int ess_count = 0;
  const double band = 0.02 * std::abs(step);

  for (int k = 0; k < steps; ++k) {
    const double time = k * dt;
    const GimbalState measured = plant.measure();
    const ControllerOutput command = controller.update(measured, step, 0.0, dt);
    plant.step(command.voltage, dt);
    const double theta = plant.truth().pan.theta;
    peak = std::max(peak, theta);
    const double fraction = theta / step;
    if (t10 < 0.0 && fraction >= 0.1) {
      t10 = time + dt;
    }
    if (t90 < 0.0 && fraction >= 0.9) {
      t90 = time + dt;
    }
    if (std::abs(theta - step) > band) {
      last_outside = time + dt;
    }
    if (time + dt >= duration - 0.5) {
      ess_sum += theta - step;
      ++ess_count;
    }
  }

  StepStats stats;
  stats.reached_90 = t10 >= 0.0 && t90 >= 0.0;
  stats.rise_time = stats.reached_90 ? (t90 - t10) : 1.0e9;
  stats.overshoot_percent = step > 0.0 ? std::max(0.0, (peak - step) / step * 100.0) : 0.0;
  stats.settling_time = last_outside;
  stats.stayed_settled = std::abs(plant.truth().pan.theta - step) <= band;
  stats.steady_state_error = ess_count > 0 ? std::abs(ess_sum / ess_count) : 1.0e9;
  return stats;
}

}  // namespace

TEST(Plant, PositiveVoltageIncreasesPanRate) {
  PlantConfig config = testPlantConfig();
  config.pan.coulomb = 0.0;
  GimbalPlant plant(config, 1);
  GimbalCommand command;
  command.pan_voltage = 8.0;
  for (int i = 0; i < 400; ++i) {
    plant.step(command, 0.001);
  }
  EXPECT_GT(plant.truth().pan.omega, 0.2);
  EXPECT_GT(plant.truth().pan.theta, 0.0);
  EXPECT_NEAR(plant.truth().tilt.theta, 0.0, 1e-12);
}

TEST(Controller, PositionStepMeetsClassicalSpecs) {
  PlantConfig plant = testPlantConfig();
  plant.pan.coulomb = 0.0;
  plant.tilt.coulomb = 0.0;
  const StepStats stats = runPanStep(plant, testControllerConfig(), 0.25, 4.0, 0.001);
  EXPECT_TRUE(stats.reached_90);
  EXPECT_LT(stats.rise_time, 1.0);
  EXPECT_LT(stats.overshoot_percent, 20.0);
  EXPECT_LT(stats.settling_time, 2.5);
  EXPECT_TRUE(stats.stayed_settled);
  EXPECT_LT(stats.steady_state_error, 0.01);
}

TEST(Controller, ShippedYamlTuningSettlesWithFriction) {
  const SimConfig config = loadConfig(std::string(GIMBAL_TRACKER_SOURCE_DIR) + "/config/gimbal_tracker.yaml");
  PlantConfig plant = config.plant;
  plant.encoder_noise_std = 0.0;
  plant.gyro_noise_std = 0.0;
  const StepStats stats = runPanStep(plant, config.controller, config.analysis.step_rad,
                                      config.analysis.step_duration_s, config.analysis.step_dt_s);
  EXPECT_TRUE(stats.reached_90);
  EXPECT_LT(stats.rise_time, 1.2);
  EXPECT_LT(stats.overshoot_percent, 25.0);
  EXPECT_LT(stats.settling_time, 3.0);
  EXPECT_TRUE(stats.stayed_settled);
  EXPECT_LT(stats.steady_state_error, 0.02);
}

TEST(Controller, PanErrorTakesTheShortWrap) {
  AxisControllerConfig config = testAxisController();
  config.wrap_error = true;
  AxisController wrapped(config);
  AxisInput input;
  input.theta_meas = 3.0;
  input.theta_ref = -3.0;
  const AxisOutput short_way = wrapped.update(input, 0.001);
  EXPECT_NEAR(short_way.position_error, wrapPi(-6.0), 1e-12);
  EXPECT_GT(short_way.rate_command, 0.0);

  config.wrap_error = false;
  AxisController unwrapped(config);
  const AxisOutput long_way = unwrapped.update(input, 0.001);
  EXPECT_NEAR(long_way.position_error, -6.0, 1e-12);
  EXPECT_LT(long_way.rate_command, 0.0);
}

TEST(Controller, RateCommandIsSaturated) {
  AxisController controller(testAxisController());
  AxisInput input;
  input.theta_ref = 2.0;
  const AxisOutput output = controller.update(input, 0.001);
  EXPECT_NEAR(output.rate_command, controller.config().rate_limit, 1e-12);
  EXPECT_LE(std::abs(output.voltage), controller.config().voltage_limit + 1e-12);
}

TEST(Controller, BackCalculationBoundsTheRateIntegrator) {
  auto run = [](double kaw) {
    PlantConfig plant_config = testPlantConfig();
    plant_config.pan.coulomb = 0.0;
    ControllerConfig controller_config = testControllerConfig();
    controller_config.pan.voltage_limit = 1.5;
    controller_config.pan.rate.kaw = kaw;
    controller_config.pan.rate_kff = 0.0;
    controller_config.pan.position_kff = 0.0;
    controller_config.pan.rate_limit = 5.0;
    GimbalPlant plant(plant_config, 1);
    GimbalController controller(controller_config);
    double peak_integral = 0.0;
    for (int i = 0; i < 1500; ++i) {
      const GimbalState measured = plant.measure();
      const ControllerOutput command = controller.update(measured, 1.2, 0.0, 0.001);
      EXPECT_LE(std::abs(command.voltage.pan_voltage), 1.5 + 1e-12);
      plant.step(command.voltage, 0.001);
      peak_integral = std::max(peak_integral, std::abs(controller.panRateIntegral()));
    }
    return peak_integral;
  };

  const double windup = run(0.0);
  const double bounded = run(15.0);
  EXPECT_GT(windup, 40.0);
  EXPECT_LT(bounded, 0.5 * windup);
}

TEST(Controller, RateFeedforwardShrinksRampLag) {
  auto lag = [](double position_kff) {
    PlantConfig plant_config = testPlantConfig();
    plant_config.pan.coulomb = 0.0;
    plant_config.tilt.coulomb = 0.0;
    ControllerConfig controller_config = testControllerConfig();
    controller_config.pan.position_kff = position_kff;
    GimbalPlant plant(plant_config, 1);
    GimbalController controller(controller_config);
    const double omega = 0.4;
    const double dt = 0.001;
    const int steps = 4000;
    double error_sum = 0.0;
    int count = 0;
    for (int k = 0; k < steps; ++k) {
      const double time = k * dt;
      const double reference = omega * time;
      const GimbalState measured = plant.measure();
      const ControllerOutput command = controller.update(measured, reference, 0.0, dt, omega, 0.0);
      plant.step(command.voltage, dt);
      if (time > 2.0) {
        error_sum += std::abs(wrapPi(reference - plant.truth().pan.theta));
        ++count;
      }
    }
    return error_sum / static_cast<double>(count);
  };

  const double with_ff = lag(1.0);
  const double without_ff = lag(0.0);
  EXPECT_LT(with_ff, 0.05);
  EXPECT_LT(with_ff, 0.45 * without_ff);
}
