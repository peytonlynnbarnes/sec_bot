cmake_minimum_required(VERSION 3.5)
project(ball_tracker)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED) 

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  OpenCV
  Eigen3
  Pangolin
  sensor_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()