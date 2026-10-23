#include "ros_helpers.hpp"

#include <algorithm>

namespace {

class SensorNode : public rclcpp::Node {
 public:
  SensorNode() : Node("sensor"), noise_(1) {
    cfg_ = gimbal_tracker_ros::loadNodeConfig(*this);
    noise_.reseed(cfg_.sim.ros.sensor_seed);
    pub_ = create_publisher<gimbal_tracker_msgs::msg::SensorMeasurement>("/sensor/measurement", 10);
    sub_ = create_subscription<gimbal_tracker_msgs::msg::TargetTruth>(
        "/target/truth", 10, [this](gimbal_tracker_msgs::msg::TargetTruth::SharedPtr msg) { onTruth(msg); });
    RCLCPP_INFO(get_logger(), "Sensor mode '%s'", cfg_.measurement.c_str());
  }

 private:
  void onTruth(const gimbal_tracker_msgs::msg::TargetTruth::SharedPtr& msg) {
    const gimbal_tracker::Vec3 position = gimbal_tracker_ros::fromPoint(msg->position);
    gimbal_tracker_msgs::msg::SensorMeasurement out;
    out.header = msg->header;
    out.covariance.fill(0.0);
    const auto& est = cfg_.sim.estimator;
    if (cfg_.measurement == "bearing_range") {
      const gimbal_tracker::LookAngles look = gimbal_tracker::lookAngles(cfg_.sim.lead.muzzle, position);
      out.kind = gimbal_tracker_msgs::msg::SensorMeasurement::BEARING_RANGE;
      out.z[0] = gimbal_tracker::wrapPi(look.azimuth + noise_.sample(est.azimuth_std));
      out.z[1] = look.elevation + noise_.sample(est.elevation_std);
      out.z[2] = std::max(0.05, look.range + noise_.sample(est.range_std));
      out.covariance[0] = est.azimuth_std * est.azimuth_std;
      out.covariance[4] = est.elevation_std * est.elevation_std;
      out.covariance[8] = est.range_std * est.range_std;
    } else {
      out.kind = gimbal_tracker_msgs::msg::SensorMeasurement::CARTESIAN;
      out.z[0] = position.x + noise_.sample(est.cartesian_std);
      out.z[1] = position.y + noise_.sample(est.cartesian_std);
      out.z[2] = position.z + noise_.sample(est.cartesian_std);
      const double variance = est.cartesian_std * est.cartesian_std;
      out.covariance[0] = variance;
      out.covariance[4] = variance;
      out.covariance[8] = variance;
    }
    pub_->publish(out);
  }

  gimbal_tracker_ros::NodeConfig cfg_;
  gimbal_tracker::GaussianNoise noise_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::SensorMeasurement>::SharedPtr pub_;
  rclcpp::Subscription<gimbal_tracker_msgs::msg::TargetTruth>::SharedPtr sub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SensorNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("sensor"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
