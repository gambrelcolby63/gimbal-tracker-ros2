#pragma once

#include "gimbal_tracker/gimbal_tracker.hpp"
#include "gimbal_tracker_msgs/msg/gimbal_command.hpp"
#include "gimbal_tracker_msgs/msg/gimbal_state.hpp"
#include "gimbal_tracker_msgs/msg/sensor_measurement.hpp"
#include "gimbal_tracker_msgs/msg/target_estimate.hpp"
#include "gimbal_tracker_msgs/msg/target_truth.hpp"
#include "gimbal_tracker_msgs/msg/track_setpoint.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

namespace gimbal_tracker_ros {

struct NodeConfig {
  gimbal_tracker::SimConfig sim;
  double rate_hz{200.0};
  std::string measurement{"cartesian"};
};

inline NodeConfig loadNodeConfig(rclcpp::Node& node) {
  NodeConfig out;
  const std::string path = node.declare_parameter<std::string>("config", "");
  if (path.empty()) {
    throw std::runtime_error("ROS parameter 'config' must point at gimbal_tracker.yaml");
  }
  out.sim = gimbal_tracker::loadConfig(path);
  out.rate_hz = node.declare_parameter<double>("rate_hz", out.sim.ros.rate_hz);
  out.measurement = node.declare_parameter<std::string>("measurement", out.sim.ros.measurement);
  if (!(out.rate_hz > 0.0)) {
    throw std::runtime_error("rate_hz must be positive");
  }
  if (out.measurement != "cartesian" && out.measurement != "bearing_range") {
    throw std::runtime_error("measurement must be 'cartesian' or 'bearing_range'");
  }
  return out;
}

inline std::chrono::nanoseconds periodOf(double rate_hz) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz));
}

inline geometry_msgs::msg::Point toPoint(const gimbal_tracker::Vec3& value) {
  geometry_msgs::msg::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

inline geometry_msgs::msg::Vector3 toVector3(const gimbal_tracker::Vec3& value) {
  geometry_msgs::msg::Vector3 vector;
  vector.x = value.x;
  vector.y = value.y;
  vector.z = value.z;
  return vector;
}

inline gimbal_tracker::Vec3 fromPoint(const geometry_msgs::msg::Point& point) {
  return {point.x, point.y, point.z};
}

inline gimbal_tracker::Vec3 fromVector3(const geometry_msgs::msg::Vector3& vector) {
  return {vector.x, vector.y, vector.z};
}

// Pan is yaw about +z. Tilt is elevation, positive up, which is negative ROS pitch.
inline geometry_msgs::msg::Quaternion boresightQuaternion(double pan, double tilt) {
  tf2::Quaternion q_pan;
  tf2::Quaternion q_tilt;
  q_pan.setRPY(0.0, 0.0, pan);
  q_tilt.setRPY(0.0, -tilt, 0.0);
  const tf2::Quaternion q = q_pan * q_tilt;
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

inline visualization_msgs::msg::Marker baseMarker(const rclcpp::Time& stamp, const std::string& ns,
                                                  int type) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = "turret";
  marker.ns = ns;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  return marker;
}

}  // namespace gimbal_tracker_ros
