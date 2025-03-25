#include "ros2_orb_slam3/common.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ========== MONOCULAR MODE IMPLEMENTATION ==========

MonocularMode::MonocularMode() : Node("mono_node_cpp") {
    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 Monocular Node Started");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");

    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();

    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set") {
        vocFilePath = homeDir + "/ros2_ws/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt";
        settingsFilePath = homeDir + "/ros2_ws/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName: %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file: %s", vocFilePath.c_str());

    subexperimentconfigName = "/mono_py_driver/experiment_settings";
    pubconfigackName = "/mono_py_driver/exp_settings_ack";
    subImgMsgName = "/mono_py_driver/img_msg";
    subTimestepMsgName = "/mono_py_driver/timestep_msg";

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    // Initializes TF Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

MonocularMode::~MonocularMode() {
    if (pAgent) pAgent->Shutdown();
}

void MonocularMode::experimentSetting_callback(const std_msgs::msg::String &msg) {
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", experimentConfig.c_str());

    auto message = std_msgs::msg::String();
    message.data = "ACK";
    configAck_publisher_->publish(message);

    initializeVSLAM(experimentConfig);
}

void MonocularMode::initializeVSLAM(std::string &configString) {
    if (vocFilePath.empty() || settingsFilePath.empty()) {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }
    settingsFilePath += configString + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = false;

    pAgent = std::make_shared<ORB_SLAM3::System>(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
}

void MonocularMode::Timestep_callback(const std_msgs::msg::Float64 &msg) {
    timeStep = msg.data;
}

void MonocularMode::Img_callback(const sensor_msgs::msg::Image &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }

    if (pAgent) {
        // Track and get pose
        Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);

        // Publish TF if tracking is valid
        if (!Tcw.matrix().isIdentity()) {
            Eigen::Quaternionf q(Tcw.rotationMatrix());
            Eigen::Vector3f t = Tcw.translation();

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "map";
            transform.child_frame_id = "base_link";

            transform.transform.translation.x = t.x();
            transform.transform.translation.y = t.y();
            transform.transform.translation.z = t.z();
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(transform);
        }
    }
}

// ========== STEREO MODE IMPLEMENTATION ==========

StereoMode::StereoMode() : Node("stereo_node_cpp") {
    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 Stereo Node Started");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");
    this->declare_parameter("left_image_topic", "/camera/left/image_raw");
    this->declare_parameter("right_image_topic", "/camera/right/image_raw");

    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();
    leftImageTopic = this->get_parameter("left_image_topic").as_string();
    rightImageTopic = this->get_parameter("right_image_topic").as_string();

    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set") {
        vocFilePath = homeDir + "/ros2_ws/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt";
        settingsFilePath = homeDir + "/ros2_ws/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo/";
    }

    // Initializes TF Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    initializeParams();
    initializeVSLAM();
    RCLCPP_INFO(this->get_logger(), "StereoMode node initialized");
}

StereoMode::~StereoMode() {
    if (pAgent) pAgent->Shutdown();
}

void StereoMode::initializeParams() {
    subLeftImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        leftImageTopic, 1, std::bind(&StereoMode::LeftImg_callback, this, _1));

    subRightImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        rightImageTopic, 1, std::bind(&StereoMode::RightImg_callback, this, _1));

    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/stereo_py_driver/timestep_msg", 1, std::bind(&StereoMode::Timestep_callback, this, _1));
}

void StereoMode::initializeVSLAM() {
    if (vocFilePath.empty() || settingsFilePath.empty()) {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    settingsFilePath += "stereo.yaml";
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::STEREO;
    enablePangolinWindow = true;
    enableOpenCVWindow = false;

    pAgent = std::make_shared<ORB_SLAM3::System>(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
}

void StereoMode::Timestep_callback(const std_msgs::msg::Float64 &msg) {
    timeStep = msg.data;
}

void StereoMode::LeftImg_callback(const sensor_msgs::msg::Image &msg) {
    try {
        leftImg = cv_bridge::toCvCopy(msg)->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Left image cv_bridge error");
        return;
    }
    if (!rightImg.empty()) runStereo();
}

void StereoMode::RightImg_callback(const sensor_msgs::msg::Image &msg) {
    try {
        rightImg = cv_bridge::toCvCopy(msg)->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Right image cv_bridge error");
        return;
    }
    if (!leftImg.empty()) runStereo();
}

void StereoMode::runStereo() {
    if (!pAgent || timeStep <= 0) return;
    
    // Track and get pose
    Sophus::SE3f Tcw = pAgent->TrackStereo(leftImg, rightImg, timeStep);

    // Publish TF if tracking is valid
    if (!Tcw.matrix().isIdentity()) {
        Eigen::Quaternionf q(Tcw.rotationMatrix());
        Eigen::Vector3f t = Tcw.translation();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = t.x();
        transform.transform.translation.y = t.y();
        transform.transform.translation.z = t.z();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }
}