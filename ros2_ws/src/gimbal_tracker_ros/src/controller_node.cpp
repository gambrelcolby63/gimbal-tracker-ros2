#include "ros_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace {

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : Node("controller") {
    cfg_ = gimbal_tracker_ros::loadNodeConfig(*this);
    controller_ = std::make_unique<gimbal_tracker::GimbalController>(cfg_.sim.controller);
    command_pub_ = create_publisher<gimbal_tracker_msgs::msg::GimbalCommand>("/gimbal/command", 10);
    setpoint_pub_ = create_publisher<gimbal_tracker_msgs::msg::TrackSetpoint>("/gimbal/setpoint", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/markers/lead", 10);
    estimate_sub_ = create_subscription<gimbal_tracker_msgs::msg::TargetEstimate>(
        "/target/estimate", 10,
        [this](gimbal_tracker_msgs::msg::TargetEstimate::SharedPtr msg) { estimate_ = *msg; have_estimate_ = true; });
    state_sub_ = create_subscription<gimbal_tracker_msgs::msg::GimbalState>(
        "/gimbal/state", 10, [this](gimbal_tracker_msgs::msg::GimbalState::SharedPtr msg) { onState(msg); });
    RCLCPP_INFO(get_logger(), "Controller projectile speed %.1f m/s, gravity %.3f m/s^2",
                cfg_.sim.lead.projectile_speed, cfg_.sim.lead.gravity);
  }

 private:
  gimbal_tracker::LeadRequest leadRequest(const gimbal_tracker_msgs::msg::TargetEstimate& estimate) const {
    gimbal_tracker::LeadRequest request;
    request.muzzle = cfg_.sim.lead.muzzle;
    request.target_position = gimbal_tracker_ros::fromPoint(estimate.position);
    request.target_velocity = gimbal_tracker_ros::fromVector3(estimate.velocity);
    request.target_acceleration = gimbal_tracker_ros::fromVector3(estimate.acceleration);
    request.projectile_speed = cfg_.sim.lead.projectile_speed;
    request.gravity = cfg_.sim.lead.gravity;
    request.max_iterations = cfg_.sim.lead.max_iterations;
    request.tolerance = cfg_.sim.lead.tolerance;
    return request;
  }

  void onState(const gimbal_tracker_msgs::msg::GimbalState::SharedPtr& state) {
    const rclcpp::Time stamp = now();
    double pan_ref = 0.0;
    double tilt_ref = 0.0;
    gimbal_tracker::LeadSolution lead;
    if (have_estimate_) {
      lead = gimbal_tracker::solveLead(leadRequest(estimate_));
      if (lead.valid) {
        pan_ref = lead.azimuth;
        tilt_ref = lead.elevation;
      } else {
        const gimbal_tracker::LookAngles los = gimbal_tracker::lookAngles(
            cfg_.sim.lead.muzzle, gimbal_tracker_ros::fromPoint(estimate_.position));
        pan_ref = los.azimuth;
        tilt_ref = los.elevation;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Lead solution unavailable (%s); pointing along LOS",
                             lead.reason.c_str());
      }
    }

    gimbal_tracker::GimbalState measured;
    measured.pan.theta = state->pan;
    measured.pan.omega = state->pan_rate;
    measured.tilt.theta = state->tilt;
    measured.tilt.omega = state->tilt_rate;
    const auto output = controller_->update(measured, pan_ref, tilt_ref, 1.0 / cfg_.rate_hz);

    gimbal_tracker_msgs::msg::GimbalCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = "turret";
    command.pan_voltage = output.voltage.pan_voltage;
    command.tilt_voltage = output.voltage.tilt_voltage;
    command_pub_->publish(command);

    gimbal_tracker_msgs::msg::TrackSetpoint setpoint;
    setpoint.header = command.header;
    setpoint.pan = pan_ref;
    setpoint.tilt = tilt_ref;
    setpoint.time_of_flight = lead.time_of_flight;
    setpoint.intercept = gimbal_tracker_ros::toPoint(lead.intercept);
    setpoint.valid = lead.valid;
    setpoint.detail = lead.valid ? "ok" : lead.reason;
    setpoint_pub_->publish(setpoint);
    publishLeadMarker(stamp, lead);
  }

  void publishLeadMarker(const rclcpp::Time& stamp, const gimbal_tracker::LeadSolution& lead) {
    const int stride = std::max(1, static_cast<int>(std::lround(cfg_.rate_hz / 30.0)));
    if (++marker_tick_ % stride != 0) {
      return;
    }
    auto line = gimbal_tracker_ros::baseMarker(stamp, "lead_line", visualization_msgs::msg::Marker::LINE_STRIP);
    auto point = gimbal_tracker_ros::baseMarker(stamp, "lead_point", visualization_msgs::msg::Marker::SPHERE);
    if (!lead.valid) {
      line.action = visualization_msgs::msg::Marker::DELETE;
      point.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(line);
      point.id = 1;
      marker_pub_->publish(point);
      return;
    }
    geometry_msgs::msg::Point origin;
    line.points.push_back(origin);
    line.points.push_back(gimbal_tracker_ros::toPoint(lead.intercept));
    line.scale.x = 0.15;
    line.color.r = 0.98f;
    line.color.g = 0.85f;
    line.color.b = 0.2f;
    marker_pub_->publish(line);

    point.id = 1;
    point.pose.position = gimbal_tracker_ros::toPoint(lead.intercept);
    point.scale.x = point.scale.y = point.scale.z = 1.2;
    point.color.r = 0.2f;
    point.color.g = 0.75f;
    point.color.b = 0.35f;
    marker_pub_->publish(point);
  }

  gimbal_tracker_ros::NodeConfig cfg_;
  std::unique_ptr<gimbal_tracker::GimbalController> controller_;
  gimbal_tracker_msgs::msg::TargetEstimate estimate_;
  bool have_estimate_{false};
  int marker_tick_{0};
  rclcpp::Publisher<gimbal_tracker_msgs::msg::GimbalCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<gimbal_tracker_msgs::msg::TrackSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<gimbal_tracker_msgs::msg::TargetEstimate>::SharedPtr estimate_sub_;
  rclcpp::Subscription<gimbal_tracker_msgs::msg::GimbalState>::SharedPtr state_sub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControllerNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
