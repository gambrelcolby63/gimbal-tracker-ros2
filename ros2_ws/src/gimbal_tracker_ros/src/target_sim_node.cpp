#include "ros_helpers.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

class TargetSimNode : public rclcpp::Node {
 public:
  TargetSimNode() : Node("target_sim") {
    cfg_ = gimbal_tracker_ros::loadNodeConfig(*this);
    trajectory_ = std::make_unique<gimbal_tracker::TargetTrajectory>(cfg_.sim.target);
    pub_ = create_publisher<gimbal_tracker_msgs::msg::TargetTruth>("/target/truth", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/markers/target", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticFrame();
    timer_ = create_wall_timer(gimbal_tracker_ros::periodOf(cfg_.rate_hz), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "Target trajectory '%s' at %.1f Hz", cfg_.sim.target.trajectory.c_str(),
                cfg_.rate_hz);
  }

 private:
  void publishStaticFrame() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "turret";
    tf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(tf);
  }

  void onTimer() {
    const double dt = 1.0 / cfg_.rate_hz;
    const gimbal_tracker::TargetState state = trajectory_->at(time_s_);
    gimbal_tracker_msgs::msg::TargetTruth msg;
    msg.header.stamp = now();
    msg.header.frame_id = "turret";
    msg.position = gimbal_tracker_ros::toPoint(state.position);
    msg.velocity = gimbal_tracker_ros::toVector3(state.velocity);
    msg.acceleration = gimbal_tracker_ros::toVector3(state.acceleration);
    pub_->publish(msg);

    const int stride = std::max(1, static_cast<int>(std::lround(cfg_.rate_hz / 30.0)));
    if (tick_ % stride == 0) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg.header.stamp;
      tf.header.frame_id = "turret";
      tf.child_frame_id = "target";
      tf.transform.translation.x = state.position.x;
      tf.transform.translation.y = state.position.y;
      tf.transform.translation.z = state.position.z;
      tf.transform.rotation.w = 1.0;
      tf_->sendTransform(tf);

      auto marker = gimbal_tracker_ros::baseMarker(msg.header.stamp, "target",
                                                   visualization_msgs::msg::Marker::SPHERE);
      marker.pose.position = msg.position;
      marker.scale.x = marker.scale.y = marker.scale.z = 2.0;
      marker.color.r = 0.95f;
      marker.color.g = 0.45f;
      marker.color.b = 0.1f;
      marker_pub_->publish(marker);
    }
    time_s_ += dt;
    ++tick_;
  }

  gimbal_tracker_ros::NodeConfig cfg_;
  std::unique_ptr<gimbal_tracker::TargetTrajectory> trajectory_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::TargetTruth>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  double time_s_{0.0};
  int tick_{0};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TargetSimNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("target_sim"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
