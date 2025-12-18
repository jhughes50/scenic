/*!
* @Author: Jason Hughes
* @Date: December 2025
*
* @About: thin wrapper around opencv kmeans
*/

#pragma once

#include <unordered_set>
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
    std::vector<cv::Point> points;
};

struct KMeans
{
    static KMeansOutput Cluster(const cv::Mat mask, int k);
    static std::unordered_map<uchar, std::unordered_set<uchar>> findAdjacentRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k);
    int getNumRegions(); // TODO
};
}
