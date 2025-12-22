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
struct KMeansOutput
{
    std::vector<cv::Point> centroids;
    cv::Mat voronoi;
    std::vector<cv::Point> points;
};

struct AdjacencyOutput
{
    std::unordered_map<uchar, std::unordered_set<uchar>> adjacency;
    std::unordered_map<uchar, cv::Point> centroids;
};

struct KMeans
{
    static KMeansOutput Cluster(const cv::Mat mask, int k);
    static AdjacencyOutput ConnectRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k);
    int getNumRegions(); // TODO
};
}
