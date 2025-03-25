// stereo_node.cpp
#include <iostream>
#include <signal.h>
#include <string>

// Suppress warnings from third-party headers.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wreorder"

// Include ORB_SLAM3 system header (and its dependencies).
#include "System.h"

#pragma GCC diagnostic pop

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Include Sophus for SE3f types and conversion.
#include <sophus/se3.hpp>

// Global flag for clean shutdown on SIGINT.
volatile bool g_running = true;
void signalHandler(int signum)
{
    g_running = false;
}

int main(int argc, char **argv)
{
    // Register SIGINT handler.
    signal(SIGINT, signalHandler);

    if (argc < 3) {
        std::cerr << "Usage: stereo_node <vocabulary_file> <settings_file>" << std::endl;
        return 1;
    }

    std::string vocabFile   = argv[1];
    std::string settingsFile = argv[2];

    // Create the ORB_SLAM3 system in stereo mode.
    // (The last boolean parameter controls whether to use viewer.)
    ORB_SLAM3::System SLAM(vocabFile, settingsFile, ORB_SLAM3::System::STEREO, true);

    // Main processing loop.
    while (g_running)
    {
        // Acquire stereo images.
        cv::Mat leftImg  = cv::imread("left.png",  cv::IMREAD_GRAYSCALE);
        cv::Mat rightImg = cv::imread("right.png", cv::IMREAD_GRAYSCALE);

        if (leftImg.empty() || rightImg.empty())
        {
            std::cerr << "Error: Could not load input images." << std::endl;
            break;
        }

        // Process the stereo images.
        // Note: TrackStereo now returns a Sophus::SE3f object.
        Sophus::SE3f Tcw = SLAM.TrackStereo(leftImg, rightImg, static_cast<double>(cv::getTickCount()));

        // Use the transformation (e.g., print the pose transformation matrix).
        std::cout << "Current Pose:\n" << Tcw.matrix() << std::endl;

        // Optional: show the left image.
        cv::imshow("Left Image", leftImg);
        if (cv::waitKey(30) == 27) // exit if ESC is pressed.
            break;
    }

    // Shutdown and release resources.
    SLAM.Shutdown();

    return 0;
}
