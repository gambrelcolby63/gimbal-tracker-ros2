cmake_minimum_required(VERSION 3.16)
project(gimbal_tracker_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gimbal_tracker_msgs REQUIRED)

# The ROS-free library lives next to ros2_ws/, not inside the colcon src tree.
get_filename_component(GIMBAL_TRACKER_LIBRARY_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../library" ABSOLUTE)
add_subdirectory(${GIMBAL_TRACKER_LIBRARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/gimbal_tracker_core)

set(NODE_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  gimbal_tracker_msgs
)

foreach(node target_sim_node sensor_node estimator_node controller_node plant_node)
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} ${NODE_DEPENDENCIES})
  target_link_libraries(${node} gimbal_tracker)
  target_include_directories(${node} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endforeach()

install(TARGETS target_sim_node sensor_node estimator_node controller_node plant_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/gimbal_tracker.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
