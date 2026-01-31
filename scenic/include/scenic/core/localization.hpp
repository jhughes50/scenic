/*!
* Jason Hughes
* September 2025
*
* Multi-thread the localization of the image
*/

#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "scenic/graphing/node.hpp"
#include "scenic/utils/transforms.hpp"

namespace Scenic
{
struct BufferLocalization
{
    cv::Mat& coords;
    cv::Mat& intrinsics;
    cv::Mat& distortion;
    Eigen::Isometry3d& pose;

    Transforms transforms;

    BufferLocalization(cv::Mat& c, cv::Mat& k, cv::Mat& d, Eigen::Isometry3d& p);
    BufferLocalization(BufferLocalization& bl, tbb::split);

    void operator()(const tbb::blocked_range2d<int>& range);
    void join(const BufferLocalization& other);
};

void localizeNode(std::shared_ptr<Node>& node, const Eigen::Isometry3d& pose, cv::Mat K, cv::Mat D);
}
