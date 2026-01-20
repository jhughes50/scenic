/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About stitch graphs together
*/
#pragma once

#include "scenic/graphing/graph.hpp"
#include "glider/core/odometry.hpp"

namespace Scenic 
{
struct GraphWithPose
{
    GraphWithPose(Graph& g, Glider::Odometry o)
    {
        graph = std::make_shared<Graph>(g);
        odom = o;
    }
    uint64_t pid;
    std::shared_ptr<Graph> graph;
    Glider::Odometry odom;
};

struct RegionNodeMap
{
    uint64_t proposed;
    uint64_t existing;
    double distance;
};
}
