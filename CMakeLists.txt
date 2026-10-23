cmake_minimum_required(VERSION 3.16)
project(gimbal_tracker VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GIMBAL_TRACKER_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(GIMBAL_TRACKER_BUILD_APPS "Build analysis command-line tools" ON)

add_subdirectory(library)

if(GIMBAL_TRACKER_BUILD_APPS)
  foreach(app step_response estimator_mc tracking_demo imm_comparison)
    add_executable(${app} apps/${app}.cpp)
    target_link_libraries(${app} PRIVATE gimbal_tracker)
    target_compile_options(${app} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

if(GIMBAL_TRACKER_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(gimbal_tracker_tests
    tests/test_controller.cpp
    tests/test_kalman.cpp
    tests/test_lead.cpp
    tests/test_imm.cpp
  )
  target_link_libraries(gimbal_tracker_tests PRIVATE gimbal_tracker GTest::gtest_main)
  target_compile_definitions(gimbal_tracker_tests PRIVATE
    GIMBAL_TRACKER_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  target_compile_options(gimbal_tracker_tests PRIVATE -Wall -Wextra -Wpedantic)
  include(GoogleTest)
  gtest_discover_tests(gimbal_tracker_tests DISCOVERY_TIMEOUT 30)
endif()

include(GNUInstallDirs)
install(TARGETS gimbal_tracker
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES config/gimbal_tracker.yaml DESTINATION ${CMAKE_INSTALL_DATADIR}/gimbal_tracker)
if(GIMBAL_TRACKER_BUILD_APPS)
  install(TARGETS step_response estimator_mc tracking_demo imm_comparison RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
