find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

set(GIMBAL_TRACKER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(gimbal_tracker STATIC
  ${GIMBAL_TRACKER_ROOT}/src/config.cpp
  ${GIMBAL_TRACKER_ROOT}/src/controller.cpp
  ${GIMBAL_TRACKER_ROOT}/src/kalman.cpp
  ${GIMBAL_TRACKER_ROOT}/src/lead.cpp
  ${GIMBAL_TRACKER_ROOT}/src/monte_carlo.cpp
  ${GIMBAL_TRACKER_ROOT}/src/pid.cpp
  ${GIMBAL_TRACKER_ROOT}/src/plant.cpp
  ${GIMBAL_TRACKER_ROOT}/src/trajectory.cpp
)
target_compile_features(gimbal_tracker PUBLIC cxx_std_17)
target_include_directories(gimbal_tracker PUBLIC
  $<BUILD_INTERFACE:${GIMBAL_TRACKER_ROOT}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(gimbal_tracker PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(gimbal_tracker PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(gimbal_tracker PUBLIC yaml-cpp::yaml-cpp Eigen3::Eigen)
elseif(TARGET yaml-cpp)
  target_link_libraries(gimbal_tracker PUBLIC yaml-cpp Eigen3::Eigen)
else()
  message(FATAL_ERROR "yaml-cpp was found but neither yaml-cpp::yaml-cpp nor yaml-cpp is a CMake target")
endif()

if(NOT TARGET gimbal_tracker::gimbal_tracker)
  add_library(gimbal_tracker::gimbal_tracker ALIAS gimbal_tracker)
endif()
