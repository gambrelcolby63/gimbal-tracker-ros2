#include "gimbal_tracker/gimbal_tracker.hpp"

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
    std::cerr << "Usage: estimator_mc <config.yaml> <output.csv> [output.json]\n";
    return 2;
  }
  gimbal_tracker::SimConfig config;
  try {
    config = gimbal_tracker::loadConfig(argv[1]);
  } catch (const std::exception& ex) {
    return fail(ex.what());
  }

  gimbal_tracker::MonteCarloConfig monte_carlo;
  monte_carlo.model = gimbal_tracker::TargetKalmanFilter::Model::ConstantVelocity;
  monte_carlo.runs = config.analysis.mc_runs;
  monte_carlo.dt = config.analysis.mc_dt_s;
  monte_carlo.duration = config.analysis.mc_duration_s;
  monte_carlo.accel_std = config.analysis.mc_accel_std;
  monte_carlo.meas_std = config.analysis.mc_meas_std;
  monte_carlo.pos_var = config.analysis.mc_pos_var;
  monte_carlo.vel_var = config.analysis.mc_vel_var;
  monte_carlo.acc_var = config.analysis.mc_acc_var;
  monte_carlo.seed = config.analysis.mc_seed;

  const gimbal_tracker::MonteCarloResult result = gimbal_tracker::runConsistencyMonteCarlo(monte_carlo);

  std::ofstream csv(argv[2]);
  if (!csv) {
    return fail(std::string("Could not write ") + argv[2]);
  }
  csv << "t,mean_nees,mean_nis,pos_rmse,vel_rmse\n";
  csv.precision(16);
  for (std::size_t i = 0; i < result.time.size(); ++i) {
    csv << result.time[i] << "," << result.mean_nees[i] << "," << result.mean_nis[i] << ","
        << result.pos_rmse[i] << "," << result.vel_rmse[i] << "\n";
  }

  const std::string json_path = argc >= 4 ? argv[3] : std::string(argv[2]) + ".json";
  std::ofstream json(json_path);
  if (!json) {
    return fail("Could not write " + json_path);
  }
  json.precision(16);
  json << "{\n"
       << "  \"runs\": " << result.runs << ",\n"
       << "  \"state_dim\": " << result.state_dim << ",\n"
       << "  \"measurement_dim\": 3,\n"
       << "  \"dt\": " << result.dt << ",\n"
       << "  \"average_nees\": " << result.average_nees << ",\n"
       << "  \"average_nis\": " << result.average_nis << ",\n"
       << "  \"final_pos_rmse\": " << result.pos_rmse.back() << ",\n"
       << "  \"final_vel_rmse\": " << result.vel_rmse.back() << "\n"
       << "}\n";

  std::cout << "CV Monte Carlo  runs=" << result.runs << "  mean NEES=" << result.average_nees
            << "  mean NIS=" << result.average_nis
            << "  final pos RMSE=" << result.pos_rmse.back() << " m\n";
  return 0;
}
