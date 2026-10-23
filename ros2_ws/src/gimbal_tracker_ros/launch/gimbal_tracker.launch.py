"""Launch the simulated gimbal tracker.

Examples:
  ros2 launch gimbal_tracker_ros gimbal_tracker.launch.py
  ros2 launch gimbal_tracker_ros gimbal_tracker.launch.py rviz:=true
  ros2 launch gimbal_tracker_ros gimbal_tracker.launch.py measurement:=bearing_range
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("gimbal_tracker_ros")
    default_config = os.path.join(share, "config", "gimbal_tracker.yaml")
    default_rviz = os.path.join(share, "rviz", "gimbal_tracker.rviz")

    config = LaunchConfiguration("config")
    measurement = LaunchConfiguration("measurement")
    rviz = LaunchConfiguration("rviz")

    def tracker_node(executable, name):
        return Node(
            package="gimbal_tracker_ros",
            executable=executable,
            name=name,
            output="screen",
            parameters=[{"config": config, "measurement": measurement}],
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "measurement",
                default_value="cartesian",
                description="cartesian or bearing_range",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            tracker_node("target_sim_node", "target_sim"),
            tracker_node("sensor_node", "sensor"),
            tracker_node("estimator_node", "estimator"),
            tracker_node("controller_node", "controller"),
            tracker_node("plant_node", "plant"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", default_rviz],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )
