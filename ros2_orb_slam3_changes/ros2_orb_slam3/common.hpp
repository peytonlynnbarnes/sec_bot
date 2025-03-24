#ifndef ROS2_ORB_SLAM3_COMMON_HPP
#define ROS2_ORB_SLAM3_COMMON_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "System.h"
#include <memory>
#include <string>

using std::placeholders::_1;

// ---------------- MONOCULAR MODE ----------------
class MonocularMode : public rclcpp::Node {
public:
    MonocularMode();
    ~MonocularMode();

private:
    void experimentSetting_callback(const std_msgs::msg::String &msg);
    void initializeVSLAM(std::string &configString);
    void Img_callback(const sensor_msgs::msg::Image &msg);
    void Timestep_callback(const std_msgs::msg::Float64 &msg);

    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string nodeName, vocFilePath, settingsFilePath, experimentConfig, homeDir;
    bool bSettingsFromPython = false;
    double timeStep = 0;

    std::string subexperimentconfigName;
    std::string pubconfigackName;
    std::string subImgMsgName;
    std::string subTimestepMsgName;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = true;
    bool enableOpenCVWindow = false;
};

// ---------------- STEREO MODE ----------------
class StereoMode : public rclcpp::Node {
public:
    StereoMode();
    ~StereoMode();

private:
    void initializeParams();
    void initializeVSLAM();
    void Timestep_callback(const std_msgs::msg::Float64 &msg);
    void LeftImg_callback(const sensor_msgs::msg::Image &msg);
    void RightImg_callback(const sensor_msgs::msg::Image &msg);
    void runStereo();

    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string nodeName, vocFilePath, settingsFilePath, experimentConfig, homeDir;
    std::string leftImageTopic, rightImageTopic;
    double timeStep = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subLeftImgMsg_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRightImgMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

    cv::Mat leftImg, rightImg;

    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = true;
    bool enableOpenCVWindow = false;
};

#endif // ROS2_ORB_SLAM3_COMMON_HPP
