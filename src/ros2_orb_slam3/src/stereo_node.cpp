// stereo_node.cpp
// ---------------------------------------------------------------------
// IMPORTANT: To avoid multiple definition linker errors, ensure that 
// only one file provides the implementation of StereoMode. For example,
// if common.cpp also defines StereoMode’s member functions, remove it
// from your build or conditionally compile one of them.
// ---------------------------------------------------------------------

#include "ros2_orb_slam3/common.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ORB_SLAM3/System.h>
#include <sophus/se3.hpp>
#include <functional>

//* Constructor
StereoMode::StereoMode() : Node("stereo_node_cpp") {
  homeDir = getenv("HOME");

  RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 Stereo Node Started");

  // Declare parameters
  this->declare_parameter("node_name_arg", "not_given");
  this->declare_parameter("voc_file_arg", "file_not_set");
  this->declare_parameter("settings_file_path_arg", "file_path_not_set");

  nodeName = this->get_parameter("node_name_arg").as_string();
  vocFilePath = this->get_parameter("voc_file_arg").as_string();
  settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();

  if (vocFilePath == "file_not_set" || settingsFilePath == "file_path_not_set") {
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Stereo/";
  }

  RCLCPP_INFO(this->get_logger(), "nodeName: %s", nodeName.c_str());
  RCLCPP_INFO(this->get_logger(), "voc_file: %s", vocFilePath.c_str());

  subexperimentconfigName = "/stereo_py_driver/experiment_settings";
  pubconfigackName = "/stereo_py_driver/exp_settings_ack";
  subLeftImgMsgName = "/stereo_py_driver/left_img_msg";
  subRightImgMsgName = "/stereo_py_driver/right_img_msg";
  subTimestepMsgName = "/stereo_py_driver/timestep_msg";

  expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
      subexperimentconfigName, 1,
      std::bind(&StereoMode::experimentSetting_callback, this, std::placeholders::_1));
  configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
  subLeftImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      subLeftImgMsgName, 1,
      std::bind(&StereoMode::LeftImg_callback, this, std::placeholders::_1));
  subRightImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      subRightImgMsgName, 1,
      std::bind(&StereoMode::RightImg_callback, this, std::placeholders::_1));
  subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      subTimestepMsgName, 1,
      std::bind(&StereoMode::Timestep_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

StereoMode::~StereoMode() {
  if(pAgent) {
    pAgent->Shutdown();
  }
}

void StereoMode::experimentSetting_callback(const std_msgs::msg::String &msg) {
  bSettingsFromPython = true;
  experimentConfig = msg.data;
  RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", experimentConfig.c_str());

  auto message = std_msgs::msg::String();
  message.data = "ACK";
  configAck_publisher_->publish(message);

  initializeVSLAM(experimentConfig);
}

void StereoMode::initializeVSLAM(std::string &configString) {
  if (vocFilePath == "file_not_set" || settingsFilePath == "file_path_not_set") {
    RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
    rclcpp::shutdown();
    return;
  }

  settingsFilePath += configString + ".yaml";
  RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

  sensorType = ORB_SLAM3::System::STEREO;
  enablePangolinWindow = true;
  enableOpenCVWindow = true;

  pAgent = std::make_shared<ORB_SLAM3::System>(
      vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
  std::cout << "StereoMode node initialized" << std::endl;
}

void StereoMode::Timestep_callback(const std_msgs::msg::Float64 &time_msg) {
  timeStep = time_msg.data;
}

void StereoMode::LeftImg_callback(const sensor_msgs::msg::Image &msg) {
  try {
    leftImg = cv_bridge::toCvCopy(msg)->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Left image cv_bridge error");
    return;
  }

  if (!rightImg.empty())
    runStereo();
}

void StereoMode::RightImg_callback(const sensor_msgs::msg::Image &msg) {
  try {
    rightImg = cv_bridge::toCvCopy(msg)->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Right image cv_bridge error");
    return;
  }

  if (!leftImg.empty())
    runStereo();
}

void StereoMode::runStereo() {
  if (!pAgent || timeStep <= 0)
    return;

  Sophus::SE3f Tcw = pAgent->TrackStereo(leftImg, rightImg, timeStep);
  (void)Tcw;  // Suppress "set but not used" warning
  // Optional: add post-processing, publishing, etc.
}

//* main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoMode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
