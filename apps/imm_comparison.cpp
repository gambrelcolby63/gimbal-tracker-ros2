#include "gimbal_tracker/gimbal_tracker.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const std::string& message) {
  std::cerr << message << "\n";
  return 1;
}

struct Sample {
  double pos_cv{0.0};
  double pos_imm{0.0};
  double vel_cv{0.0};
  double vel_imm{0.0};
  double nees_cv{0.0};
  double nees_imm{0.0};
  double mu_cv{0.0};
  double mu_ca{0.0};
  double mu_ct{0.0};
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: imm_comparison <config.yaml> <output.csv> <output.json> [trace.csv]\n";
    return 2;
  }
  gimbal_tracker::SimConfig config;
  try {
    config = gimbal_tracker::loadConfig(argv[1]);
  } catch (const std::exception& ex) {
    return fail(ex.what());
  }

  const gimbal_tracker::TargetTrajectory trajectory(config.maneuver);
  const double dt = config.analysis.imm_dt_s;
  const int steps = static_cast<int>(std::lround(config.analysis.imm_duration_s / dt));
  const int runs = config.analysis.imm_runs;
  const double meas_std = config.analysis.imm_meas_std;
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * meas_std * meas_std;
  std::vector<Sample> totals(static_cast<std::size_t>(steps + 1));
  std::ofstream trace;
  if (argc >= 5) {
    trace.open(argv[4]);
    if (!trace) {
      return fail(std::string("Could not write ") + argv[4]);
    }
    trace << "t,truth_x,truth_y,meas_x,meas_y,cv_x,cv_y,imm_x,imm_y,mu_cv,mu_ca,mu_ct\n";
    trace.precision(10);
  }

  for (int run = 0; run < runs; ++run) {
    gimbal_tracker::GaussianNoise noise(config.analysis.imm_seed + static_cast<unsigned>(run) * 17u);
    gimbal_tracker::TargetKalmanFilter cv(gimbal_tracker::TargetKalmanFilter::Model::ConstantVelocity,
                                          config.estimator.accel_std, config.estimator.jerk_std);
    gimbal_tracker::InteractingMultipleModel imm(config.estimator);
    for (int k = 0; k <= steps; ++k) {
      const double time = k * dt;
      const gimbal_tracker::TargetState truth = trajectory.at(time);
      Eigen::Vector3d measurement;
      measurement << truth.position.x + noise.sample(meas_std), truth.position.y + noise.sample(meas_std),
          truth.position.z + noise.sample(meas_std);
      if (k == 0) {
        cv.initializeFromCartesian(measurement, config.estimator.p0_pos, config.estimator.p0_vel,
                                   config.estimator.p0_acc);
        imm.initializeFromCartesian(measurement, config.estimator.p0_pos, config.estimator.p0_vel,
                                    config.estimator.p0_acc);
      } else {
        cv.predict(dt);
        imm.predict(dt);
        cv.updateCartesian(measurement, covariance);
        imm.updateCartesian(measurement, covariance);
      }
      const auto cv_state = cv.estimate();
      const auto imm_state = imm.estimate();
      const auto probability = imm.modelProbability();
      Sample& sample = totals[static_cast<std::size_t>(k)];
      const double pos_cv = (cv_state.position - truth.position).squaredNorm();
      const double pos_imm = (imm_state.position - truth.position).squaredNorm();
      const double vel_cv = (cv_state.velocity - truth.velocity).squaredNorm();
      const double vel_imm = (imm_state.velocity - truth.velocity).squaredNorm();
      sample.pos_cv += pos_cv;
      sample.pos_imm += pos_imm;
      sample.vel_cv += vel_cv;
      sample.vel_imm += vel_imm;
      sample.nees_cv += cv.nees(truth);
      sample.nees_imm += imm.nees(truth);
      sample.mu_cv += probability(0);
      sample.mu_ca += probability(1);
      sample.mu_ct += probability(2);
      if (run == 0 && trace) {
        trace << time << "," << truth.position.x << "," << truth.position.y << "," << measurement(0) << ","
              << measurement(1) << "," << cv_state.position.x << "," << cv_state.position.y << ","
              << imm_state.position.x << "," << imm_state.position.y << "," << probability(0) << ","
              << probability(1) << "," << probability(2) << "\n";
      }
    }
  }

  std::ofstream csv(argv[2]);
  if (!csv) {
    return fail(std::string("Could not write ") + argv[2]);
  }
  csv << "t,cv_pos_rmse,imm_pos_rmse,cv_vel_rmse,imm_vel_rmse,cv_nees,imm_nees,mu_cv,mu_ca,mu_ct,kind\n";
  csv.precision(10);
  auto accumulate = [&](auto predicate, double Sample::* field) {
    double sum = 0.0;
    int count = 0;
    for (int k = 0; k <= steps; ++k) {
      const double time = k * dt;
      if (!predicate(trajectory.kindAt(time))) {
        continue;
      }
      sum += totals[static_cast<std::size_t>(k)].*field;
      ++count;
    }
    if (count == 0) {
      return 0.0;
    }
    return std::sqrt(sum / (static_cast<double>(count) * runs));
  };
  auto mean_of = [&](auto predicate, double Sample::* field) {
    double sum = 0.0;
    int count = 0;
    for (int k = 0; k <= steps; ++k) {
      if (!predicate(trajectory.kindAt(k * dt))) {
        continue;
      }
      sum += totals[static_cast<std::size_t>(k)].*field / runs;
      ++count;
    }
    return count == 0 ? 0.0 : sum / count;
  };

  const auto is_straight = [](const std::string& kind) { return kind == "straight"; };
  const auto is_turn = [](const std::string& kind) { return kind == "coordinated_turn"; };
  const auto is_speed = [](const std::string& kind) { return kind == "speed_change"; };
  const auto is_weave = [](const std::string& kind) { return kind == "weave"; };

  for (int k = 0; k <= steps; ++k) {
    const Sample& sample = totals[static_cast<std::size_t>(k)];
    const double scale = static_cast<double>(runs);
    csv << (k * dt) << "," << std::sqrt(sample.pos_cv / scale) << "," << std::sqrt(sample.pos_imm / scale) << ","
        << std::sqrt(sample.vel_cv / scale) << "," << std::sqrt(sample.vel_imm / scale) << ","
        << (sample.nees_cv / scale) << "," << (sample.nees_imm / scale) << "," << (sample.mu_cv / scale) << ","
        << (sample.mu_ca / scale) << "," << (sample.mu_ct / scale) << "," << trajectory.kindAt(k * dt) << "\n";
  }

  const double turn_cv = accumulate(is_turn, &Sample::pos_cv);
  const double turn_imm = accumulate(is_turn, &Sample::pos_imm);
  std::ofstream json(argv[3]);
  if (!json) {
    return fail(std::string("Could not write ") + argv[3]);
  }
  json.setf(std::ios::fixed);
  json.precision(6);
  json << "{\n"
       << "  \"runs\": " << runs << ",\n"
       << "  \"state_dim\": 6,\n"
       << "  \"duration_s\": " << (steps * dt) << ",\n"
       << "  \"dt_s\": " << dt << ",\n"
       << "  \"meas_std_m\": " << meas_std << ",\n"
       << "  \"turn_pos_rmse_cv\": " << turn_cv << ",\n"
       << "  \"turn_pos_rmse_imm\": " << turn_imm << ",\n"
       << "  \"straight_pos_rmse_cv\": " << accumulate(is_straight, &Sample::pos_cv) << ",\n"
       << "  \"straight_pos_rmse_imm\": " << accumulate(is_straight, &Sample::pos_imm) << ",\n"
       << "  \"speed_pos_rmse_cv\": " << accumulate(is_speed, &Sample::pos_cv) << ",\n"
       << "  \"speed_pos_rmse_imm\": " << accumulate(is_speed, &Sample::pos_imm) << ",\n"
       << "  \"weave_pos_rmse_cv\": " << accumulate(is_weave, &Sample::pos_cv) << ",\n"
       << "  \"weave_pos_rmse_imm\": " << accumulate(is_weave, &Sample::pos_imm) << ",\n"
       << "  \"turn_vel_rmse_cv\": " << accumulate(is_turn, &Sample::vel_cv) << ",\n"
       << "  \"turn_vel_rmse_imm\": " << accumulate(is_turn, &Sample::vel_imm) << ",\n"
       << "  \"mean_nees_cv\": " << mean_of([](const std::string&) { return true; }, &Sample::nees_cv) << ",\n"
       << "  \"mean_nees_imm\": " << mean_of([](const std::string&) { return true; }, &Sample::nees_imm) << ",\n"
       << "  \"ct_probability_during_turn\": " << mean_of(is_turn, &Sample::mu_ct) << ",\n"
       << "  \"ca_probability_during_speed_change\": " << mean_of(is_speed, &Sample::mu_ca) << ",\n"
       << "  \"cv_probability_during_straight\": " << mean_of(is_straight, &Sample::mu_cv) << "\n"
       << "}\n";
  std::cout << "turn position RMSE  CV " << turn_cv << " m   IMM " << turn_imm << " m\n";
  return 0;
}
