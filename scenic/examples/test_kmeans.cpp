/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About test the kmeans clustering
*/

#include "scenic/graphing/kmeans.hpp"

int main()
{
    cv::Mat mask = cv::imread("test_mask.png", cv::IMREAD_GRAYSCALE);
    int k = 6;
    Scenic::KMeansOutput output = Scenic::KMeans::Cluster(mask, k);
    std::unordered_map<uchar, std::unordered_set<uchar>> graph = Scenic::KMeans::findAdjacentRegions(output.points, output.voronoi, k);
    for (const auto& [key, vals] : graph) {
        std::cout << "Node " << (int)key << " connected to ";
        for (const auto& v : vals) {
            std::cout << (int)v << ", ";
        }
        std::cout << std::endl;
    }
    cv::Mat display;
    output.voronoi = output.voronoi * (255 / k);
    output.voronoi.convertTo(display, CV_8U);
    cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    
    // Draw cluster centers
    for (const auto& center : output.centroids) {
        cv::circle(display, cv::Point(center.x, center.y), 5, cv::Scalar(185, 128, 41), -1);  // Red filled circle
    }

    for (const auto& [key, vals] : graph) {
        for (const auto& v : vals) {
            Scenic::Point g = output.centroids[key-1];
            cv::Point kp(g.x, g.y);
            g = output.centroids[v-1];
            cv::Point tp(g.x, g.y);
            cv::line(display, kp, tp, cv::Scalar(185, 128, 41), 2);
        }
    }
    cv::imshow("output", display);
    cv::waitKey(0);
}
