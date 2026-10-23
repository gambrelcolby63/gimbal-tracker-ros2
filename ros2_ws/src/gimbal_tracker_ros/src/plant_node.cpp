#include "ros_helpers.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

class PlantNode : public rclcpp::Node {
 public:
  PlantNode() : Node("plant") {
    cfg_ = gimbal_tracker_ros::loadNodeConfig(*this);
    plant_ = std::make_unique<gimbal_tracker::GimbalPlant>(cfg_.sim.plant, cfg_.sim.ros.plant_seed);
    state_pub_ = create_publisher<gimbal_tracker_msgs::msg::GimbalState>("/gimbal/state", 10);
    truth_pub_ = create_publisher<gimbal_tracker_msgs::msg::GimbalState>("/gimbal/truth", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/markers/boresight", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    command_sub_ = create_subscription<gimbal_tracker_msgs::msg::GimbalCommand>(
        "/gimbal/command", 10, [this](gimbal_tracker_msgs::msg::GimbalCommand::SharedPtr msg) {
          command_.pan_voltage = msg->pan_voltage;
          command_.tilt_voltage = msg->tilt_voltage;
        });
    timer_ = create_wall_timer(gimbal_tracker_ros::periodOf(cfg_.rate_hz), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "Gimbal plant at %.1f Hz", cfg_.rate_hz);
  }

 private:
  static gimbal_tracker_msgs::msg::GimbalState toMsg(const gimbal_tracker::GimbalState& state,
                                                     const rclcpp::Time& stamp) {
    gimbal_tracker_msgs::msg::GimbalState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "turret";
    msg.pan = state.pan.theta;
    msg.tilt = state.tilt.theta;
    msg.pan_rate = state.pan.omega;
    msg.tilt_rate = state.tilt.omega;
    return msg;
  }

  void onTimer() {
    plant_->step(command_, 1.0 / cfg_.rate_hz);
    const rclcpp::Time stamp = now();
    state_pub_->publish(toMsg(plant_->measure(), stamp));
    const gimbal_tracker::GimbalState truth = plant_->truth();
    truth_pub_->publish(toMsg(truth, stamp));

    const int stride = std::max(1, static_cast<int>(std::lround(cfg_.rate_hz / 30.0)));
    if (++tick_ % stride != 0) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "turret";
    tf.child_frame_id = "boresight";
    tf.transform.rotation = gimbal_tracker_ros::boresightQuaternion(truth.pan.theta, truth.tilt.theta);
    tf_->sendTransform(tf);

    auto marker =
        gimbal_tracker_ros::baseMarker(stamp, "boresight", visualization_msgs::msg::Marker::ARROW);
    marker.pose.orientation = tf.transform.rotation;
    marker.scale.x = 18.0;
    marker.scale.y = 0.45;
    marker.scale.z = 0.8;
    marker.color.r = 0.2f;
    marker.color.g = 0.75f;
    marker.color.b = 0.95f;
    marker_pub_->publish(marker);
  }

  gimbal_tracker_ros::NodeConfig cfg_;
  std::unique_ptr<gimbal_tracker::GimbalPlant> plant_;
  gimbal_tracker::GimbalCommand command_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::GimbalState>::SharedPtr truth_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<gimbal_tracker_msgs::msg::GimbalCommand>::SharedPtr command_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tick_{0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PlantNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("plant"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
