/*!
* @Author: Jason Hughes
* @Date: December 2025
*
* @About: thin wrapper around opencv kmeans
*/

#pragma once

#include <opencv2/opencv.hpp>

namespace Scenic
{

struct Point
{
    int x;
    int y;
};

struct KMeansOutput
{
    std::vector<Point> centroids;
    cv::Mat voronoi;
};

struct KMeans
{
    KMeansOutput operator()(const cv::Mat mask, int k);
};
}
