#include "ros_helpers.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <memory>

namespace {

class EstimatorNode : public rclcpp::Node {
 public:
  EstimatorNode() : Node("estimator") {
    cfg_ = gimbal_tracker_ros::loadNodeConfig(*this);
    filter_ = std::make_unique<gimbal_tracker::TargetKalmanFilter>(
        gimbal_tracker::TargetKalmanFilter::modelFromString(cfg_.sim.estimator.model),
        cfg_.sim.estimator.accel_std, cfg_.sim.estimator.jerk_std);
    pub_ = create_publisher<gimbal_tracker_msgs::msg::TargetEstimate>("/target/estimate", 10);
    sub_ = create_subscription<gimbal_tracker_msgs::msg::SensorMeasurement>(
        "/sensor/measurement", 10,
        [this](gimbal_tracker_msgs::msg::SensorMeasurement::SharedPtr msg) { onMeasurement(msg); });
    RCLCPP_INFO(get_logger(), "Estimator model '%s'", cfg_.sim.estimator.model.c_str());
  }

 private:
  static Eigen::Matrix3d covarianceOf(const gimbal_tracker_msgs::msg::SensorMeasurement& msg) {
    Eigen::Matrix3d covariance;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        covariance(row, col) = msg.covariance[static_cast<std::size_t>(row * 3 + col)];
      }
    }
    return covariance;
  }

  void onMeasurement(const gimbal_tracker_msgs::msg::SensorMeasurement::SharedPtr& msg) {
    const Eigen::Vector3d z(msg->z[0], msg->z[1], msg->z[2]);
    const Eigen::Matrix3d covariance = covarianceOf(*msg);
    const auto& prior = cfg_.sim.estimator;
    if (!filter_->initialized()) {
      if (msg->kind == gimbal_tracker_msgs::msg::SensorMeasurement::BEARING_RANGE) {
        filter_->initializeFromBearingRange(z, prior.p0_pos, prior.p0_vel, prior.p0_acc);
      } else {
        filter_->initializeFromCartesian(z, prior.p0_pos, prior.p0_vel, prior.p0_acc);
      }
      last_stamp_ = rclcpp::Time(msg->header.stamp);
      have_stamp_ = true;
      publish(*msg);
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (have_stamp_) {
      double dt = 1.0 / cfg_.rate_hz;
      try {
        dt = (stamp - last_stamp_).seconds();
      } catch (const std::exception&) {
        dt = 1.0 / cfg_.rate_hz;
      }
      if (dt > 1.0e-4) {
        filter_->predict(std::min(dt, 0.1));
      }
    }
    last_stamp_ = stamp;
    have_stamp_ = true;
    if (msg->kind == gimbal_tracker_msgs::msg::SensorMeasurement::BEARING_RANGE) {
      filter_->updateBearingRange(z, covariance);
    } else {
      filter_->updateCartesian(z, covariance);
    }
    publish(*msg);
  }

  void publish(const gimbal_tracker_msgs::msg::SensorMeasurement& measurement) {
    const gimbal_tracker::TargetState estimate = filter_->estimate();
    gimbal_tracker_msgs::msg::TargetEstimate msg;
    msg.header = measurement.header;
    msg.model = cfg_.sim.estimator.model;
    msg.position = gimbal_tracker_ros::toPoint(estimate.position);
    msg.velocity = gimbal_tracker_ros::toVector3(estimate.velocity);
    msg.acceleration = gimbal_tracker_ros::toVector3(estimate.acceleration);
    msg.pose_velocity_covariance.fill(0.0);
    const Eigen::MatrixXd& P = filter_->covariance();
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        msg.pose_velocity_covariance[static_cast<std::size_t>(row * 6 + col)] = P(row, col);
      }
    }
    msg.nis = filter_->lastNis();
    pub_->publish(msg);
  }

  gimbal_tracker_ros::NodeConfig cfg_;
  std::unique_ptr<gimbal_tracker::TargetKalmanFilter> filter_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::TargetEstimate>::SharedPtr pub_;
  rclcpp::Subscription<gimbal_tracker_msgs::msg::SensorMeasurement>::SharedPtr sub_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EstimatorNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("estimator"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
