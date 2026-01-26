/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About stitch graphs together
*/
#pragma once

#include "scenic/core/processor_inputs.hpp"
#include "scenic/graphing/graph.hpp"
#include "glider/core/odometry.hpp"

namespace Scenic 
{
struct GraphWithPose
{
    GraphWithPose(Graph& g, Glider::Odometry o, GraphingInput& gi)
    {
        graph = std::make_shared<Graph>(g);
        odom = o;
        analysis = gi;
    }
    uint64_t pid;
    std::shared_ptr<Graph> graph;
    Glider::Odometry odom;
    GraphingInput analysis;
};

struct RegionNodeMap
{
    uint64_t proposed;
    uint64_t existing;
    double distance;
};
}
