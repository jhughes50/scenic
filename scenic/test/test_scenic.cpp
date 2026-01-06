/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About test the scenic
*/

#include <gtest/gtest.h>
#include <glog/logging.h>

#include "scenic/core/scenic.hpp"
#include "scenic/core/processor_inputs.hpp"

using namespace Scenic;



TEST(ScenicTestSuite, TestClipperViaCore)
{
    google::InitGoogleLogging("Scenic");
    Glider::Odometry odom;
    std::string model_path = "/home/jason/clipper/models";
    std::string config_path = "/home/jason/clipper/config";
    
    Scenic::Scenic scenic_core(10, model_path, config_path);
    Scenic::Text reg_input("road",
                           Scenic::GraphLevel::REGION,
                           Scenic::RegionPriority::HIGH);
    Scenic::Text obj_input("car", 
                            Scenic::GraphLevel::OBJECT, 
                            Scenic::RegionPriority::NONE);
    std::cout << "ready to start" << std::endl;
    scenic_core.setText({reg_input});
    scenic_core.start();

    std::cout << " Reading Image" << std::endl;
    cv::Mat img = cv::imread("../test/test.png", cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(), 0.25, 0.25);

    std::cout << "Pushing to Scenic Core" << std::endl;
    scenic_core.push(img, odom);
    std::cout << "DONE" << std::endl;
    auto wake_time = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    std::this_thread::sleep_until(wake_time);
    Scenic::Graph graph = scenic_core.getGraph();
    for (const auto [key, edge] : graph.getEdges()) {
        std::cout << "Node: " << key.first << " <==> Node: " << key.second << " with score: " << edge->getScore() << std::endl;
    }
    cv::Mat display = Scenic::Graph::DrawGraph(graph, img);
    cv::imshow("graph", display);
    cv::waitKey(0);
}

//TEST(ScenicTestSuite, TestGraphTraversability)
//{
//    Glider::Odometry odom;
//    std::string model_path = "/home/jason/clipper/models";
//    std::string config_path = "/home/jason/clipper/config";
//
//    Scenic::Scenic scenic_core(10, model_path, config_path);
//    Scenic::Text reg_input("road",
//                           Scenic::GraphLevel::REGION,
//                           Scenic::RegionPriority::HIGH);
//    Scenic::Text obj_input("car", 
//                            Scenic::GraphLevel::OBJECT, 
//                            Scenic::RegionPriority::NONE);
//    scenic_core.setText({reg_input, obj_input});
//    scenic_core.start();
//
//    cv::Mat img = cv::imread("../test/test.png", cv::IMREAD_COLOR);
//    cv::resize(img, img, cv::Size(), 0.25, 0.25);
//
//    scenic_core.push(img, odom);
//
//    auto wake_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
//    std::this_thread::sleep_until(wake_time);
//
//    Scenic::Graph graph = scenic_core.getGraph();
//
//    for (const auto [key, edge] : graph.getEdges()) {
//        std::cout << "Node: " << key.first << " <==> Node: " << key.second << " with score: " << edge->getScore() << std::endl;
//    }
//}
