/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About test the kmeans clustering
*/

#include "scenic/graphing/kmeans.hpp"

int main()
{
    cv::Mat mask = cv::imread("test4_mask.png", cv::IMREAD_GRAYSCALE);

    Scenic::KMeans kmeans;

    Scenic::KMeansOutput output = kmeans(mask, 3);
    cv::Mat display;
    output.voronoi.convertTo(display, CV_8U);
    cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    
    // Draw cluster centers
    for (const auto& center : output.centroids) {
        cv::circle(display, cv::Point(center.x, center.y), 5, cv::Scalar(185, 128, 41), -1);  // Red filled circle
    }
    cv::imshow("output", display);
    cv::waitKey(0);
}
