#include "gimbal_tracker/gimbal_tracker.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message) {
  std::cerr << message << "\n";
  return 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: step_response <config.yaml> <output.csv> [--friction]\n";
    return 2;
  }
  bool friction = false;
  for (int i = 3; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--friction") {
      friction = true;
    } else {
      return fail("Unknown argument " + flag);
    }
  }

  gimbal_tracker::SimConfig config;
  try {
    config = gimbal_tracker::loadConfig(argv[1]);
  } catch (const std::exception& ex) {
    return fail(ex.what());
  }
  if (!friction) {
    config.plant.pan.coulomb = 0.0;
    config.plant.tilt.coulomb = 0.0;
  }
  config.plant.encoder_noise_std = 0.0;
  config.plant.gyro_noise_std = 0.0;

  std::ofstream out(argv[2]);
  if (!out) {
    return fail(std::string("Could not write ") + argv[2]);
  }
  out << "t,theta,theta_ref,omega,rate_cmd,voltage\n";
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(16);

  gimbal_tracker::GimbalPlant plant(config.plant, config.plant.seed);
  gimbal_tracker::GimbalController controller(config.controller);
  const double step = config.analysis.step_rad;
  const double dt = config.analysis.step_dt_s;
  const int steps = static_cast<int>(std::lround(config.analysis.step_duration_s / dt));
  for (int k = 0; k <= steps; ++k) {
    const double time = k * dt;
    const gimbal_tracker::GimbalState measured = plant.measure();
    const auto command = controller.update(measured, step, 0.0, dt);
    out << time << "," << plant.truth().pan.theta << "," << step << "," << plant.truth().pan.omega
        << "," << command.pan.rate_command << "," << command.voltage.pan_voltage << "\n";
    if (k == steps) {
      break;
    }
    plant.step(command.voltage, dt);
  }
  std::cout << "Wrote " << argv[2] << " (" << (friction ? "with" : "without") << " Coulomb friction)\n";
  return 0;
}
