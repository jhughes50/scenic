/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About multithreaded search for the coordinate
*/

#pragma once

#include <atomic>
#include <utility>
#include <opencv2/opencv.hpp>
#include <tbb/blocked_range2d.h>

#include "scenic/utils/geostructs.hpp"

namespace Scenic
{

class BufferSearchCoordinates
{
    public:
        BufferSearchCoordinates(const cv::Mat& coords, UTMPoint target);
        std::pair<int, int> search();

        void operator()(const tbb::blocked_range2d<int>& range) const;

    private:

        cv::Mat coords_;
        UTMPoint target_;
        double tolerance_{1.0};

        mutable std::atomic<int> row_;
        mutable std::atomic<int> col_;
        mutable std::atomic<bool> found_;
};
}
