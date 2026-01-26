/*!
* @Author: Jason Hughes
* @Date: December 2025
*
* @About: thin wrapper around opencv kmeans
*/

#pragma once

#include <cmath>
#include <string>
#include <unordered_set>
#include <opencv2/opencv.hpp>

#include "scenic/core/rectifier.hpp"

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

class KMeans
{
    public:
        KMeans() = default;
        KMeans(const std::string& path);

        KMeansOutput cluster(const cv::Mat mask, int k);
        AdjacencyOutput connectRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k);
        KMeansOutput nFixedLloyds(const cv::Mat& mask, const std::vector<cv::Point2f>& fixed_centroids, int num_new_centroids, int max_iterations, double tolerance);
        int getNumClusters(const cv::Mat& mask, int alt);

    private:
        Rectifier rectifier_;
        const int max_regions_{10};
        const double region_area_{50.0};
};
}
