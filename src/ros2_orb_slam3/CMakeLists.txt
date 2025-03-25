cmake_minimum_required(VERSION 3.10)
project(ros2_orb_slam3)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,/usr/lib/x86_64-linux-gnu")

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

# ==== Include directories ====
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${ament_INCLUDE_DIRS}
)

# ==== Transitive dependency list ====
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

# ==== ORB-SLAM3 shared library ====
add_library(orb_slam3_lib SHARED
  # Source files remain the same as in original CMakeLists.txt
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

  # Headers
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/MapDrawer.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/Viewer.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
)

ament_target_dependencies(orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto 
)

# ==== Mono Node ====
add_executable(mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

ament_target_dependencies(mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mono_node_cpp
  PUBLIC orb_slam3_lib
)

# ==== Stereo Node ====
add_executable(stereo_node_cpp
  src/stereo_node.cpp
  src/common.cpp
)

ament_target_dependencies(stereo_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(stereo_node_cpp
  PUBLIC orb_slam3_lib
)

# ==== Install Targets ====
install(TARGETS mono_node_cpp stereo_node_cpp orb_slam3_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# ==== Install Header Files ====
install(DIRECTORY include/ DESTINATION include)

# ==== Install Thirdparty .so files ====
install(FILES
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  DESTINATION lib
)

# ==== Python Script Install ====
install(PROGRAMS
  scripts/mono_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==== Export to downstream ====
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
