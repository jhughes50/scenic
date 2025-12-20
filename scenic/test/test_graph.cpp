/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About test the kmeans clustering
*/

#include <gtest/gtest.h>
#include "scenic/core/scenic.hpp"
#include "scenic/graphing/graph.hpp"
#include "scenic/core/processor_inputs.hpp"

using namespace Scenic;

bool VIZ = false;

TEST(ScenicTestSuite, TestRegionGraphInit)
{
    cv::Mat img = cv::imread("../test/test.png", cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(), 0.25, 0.25);
    cv::Mat mask = cv::imread("../test/test_mask_road.png", cv::IMREAD_GRAYSCALE);

    Scenic::GraphingInput input;
    input.masks.push_back(mask);
    input.logits.push_back(mask);
    input.image = img;
    Scenic::Text text({0,"road", 0});
    input.texts.text.push_back(text);

    Scenic::Graph graph = Scenic::Graph::RegionAnalysis(input);

    if (VIZ) {
        cv::Mat display = input.image.clone();
        for (const auto& [key, node] : graph.getNodes()) {
            std::cout << "Node : " << key << " connected to : ";
            for (const uint64_t nid : node->getConnectedIDs()) {
                std::cout << nid << ", ";
            }
            std::cout << std::endl;
        }

        for (const auto& [key, node] : graph.getNodes()) { 
            cv::Point k = graph[key]->getPixelCoordinate();
            cv::circle(display, k, 5, cv::Scalar(185, 128, 41), -1);
            for (const uint64_t nid : node->getConnectedIDs()) {
                cv::Point p = graph[nid]->getPixelCoordinate();
                cv::line(display, k, p, cv::Scalar(185, 128, 41), 2);
            }
        }
        
        cv::imshow("graph", display);
        cv::waitKey(0);
    }
}

TEST(ScenicTestSuite, TestObjectGraphInit)
{ 
    cv::Mat img = cv::imread("../test/test.png", cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(), 0.25, 0.25);
    cv::Mat mask = cv::imread("../test/test_mask_car.png", cv::IMREAD_GRAYSCALE);

    Scenic::GraphingInput input;
    input.masks.push_back(mask);
    input.logits.push_back(mask);
    input.image = img;
    Scenic::Text text({0,"car", 1});
    input.texts.text.push_back(text);

    Scenic::Graph graph = Scenic::Graph::ObjectAnalysis(input);

    cv::Mat display = input.image.clone();
    for(const auto& [key, node] : graph.getNodes()) { 
        cv::Point k = node->getPixelCoordinate();
        cv::circle(display, k, 5, cv::Scalar(128, 185, 41), -1);
    }
    cv::imshow("graph", display);
    cv::waitKey(0);
}
