/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About store the information of
* a single node in the graph
*/

#pragma once

#include <vector>

#include "scenic/graphing/edge.hpp"
#include "scenic/utils/geostructs.hpp"

namespace Scenic
{
struct Node
{
    Node() = default;
    Node(const UTMPoint& utm, const LatLonPoint& ll, const cv::Point& pixel);

    UTMPoint utm;
    LatLonPoint latlon;
    cv::Point pixel;

    int label;
    int nid;
};
}

