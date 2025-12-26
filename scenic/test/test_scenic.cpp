/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About test the scenic
*/

#include <gtest/gtest.h>

#include "scenic/core/scenic.hpp"

TEST(ScenicTestSuite, TestClipperViaCore)
{
    Glider::Odometry odom;
    std::string model_path = "/home/jason/clipper/models";
    std::string config_path = "/home/jason/clipper/config";

    Scenic::Scenic scenic_core(10, model_path, config_path);
    std::pair<std::string, Scenic::GraphLevel> p = std::make_pair("car", Scenic::GraphLevel::OBJECT);
    scenic_core.setText({p});
    scenic_core.start();

    cv::Mat img = cv::imread("../test/test.png", cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(), 0.25, 0.25);

    scenic_core.push(img, odom);

    auto wake_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::this_thread::sleep_until(wake_time);
}
