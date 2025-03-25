cmake_minimum_required(VERSION 3.10)
project(ros2_orb_slam3)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Add custom module path
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# ==== Dependencies ====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ==== Directories and Paths ====
# Ensure absolute paths are used
get_filename_component(PROJECT_ABS_PATH "${PROJECT_SOURCE_DIR}" ABSOLUTE)

# ==== Include Directories ====
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Additional ORB-SLAM3 specific include directories
include_directories(
  ${PROJECT_ABS_PATH}
  ${PROJECT_ABS_PATH}/orb_slam3
  ${PROJECT_ABS_PATH}/orb_slam3/include
  ${PROJECT_ABS_PATH}/orb_slam3/include/CameraModels
  ${PROJECT_ABS_PATH}/orb_slam3/Thirdparty
  ${PROJECT_ABS_PATH}/orb_slam3/Thirdparty/Sophus
  ${ament_INCLUDE_DIRS}
)

# ==== Transitive Dependencies ====
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
  tf2_ros
  geometry_msgs
)

# ==== Third-Party Libraries ====
# Check and set paths for third-party libraries
set(DBOW2_LIB "${PROJECT_ABS_PATH}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIB "${PROJECT_ABS_PATH}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")

# Verify library existence
if(NOT EXISTS "${DBOW2_LIB}")
  message(FATAL_ERROR "DBoW2 library not found at ${DBOW2_LIB}")
endif()

if(NOT EXISTS "${G2O_LIB}")
  message(FATAL_ERROR "g2o library not found at ${G2O_LIB}")
endif()

# ==== ORB-SLAM3 Shared Library ====
add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
)

# Link dependencies to ORB-SLAM3 library
ament_target_dependencies(orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link external libraries
target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${DBOW2_LIB}
  ${G2O_LIB}
  boost_system
  boost_serialization
  crypto 
)

# ==== Mono Node ====
add_executable(mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

ament_target_dependencies(mono_node_cpp
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mono_node_cpp
  orb_slam3_lib
)

# ==== Stereo Node ====
add_executable(stereo_node_cpp
  src/common.cpp
  src/stereo_node.cpp  # Added stereo_node.cpp
)

ament_target_dependencies(stereo_node_cpp
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(stereo_node_cpp
  orb_slam3_lib
  ${DBOW2_LIB}
  ${G2O_LIB}
)

# ==== Install Targets ====
install(TARGETS 
  mono_node_cpp 
  stereo_node_cpp 
  orb_slam3_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# ==== Install Header Files ====
install(DIRECTORY include/ DESTINATION include)

# ==== Install Thirdparty .so files ====
install(FILES
  ${DBOW2_LIB}
  ${G2O_LIB}
  DESTINATION lib
)

# ==== Python Script Install ====
install(PROGRAMS
  scripts/mono_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==== Export Dependencies ====
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()