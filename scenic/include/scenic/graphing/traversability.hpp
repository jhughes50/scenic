/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About score the traversability of each edge
*/

#pragma once

#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
            
#include "scenic/core/processor_inputs.hpp"
#include "scenic/graphing/graph.hpp"

namespace Scenic
{
struct Traversability
{
    Traversability() = default;
    static bool scoreEdge(std::shared_ptr<Edge> edge, const GraphingInput& input);
    static void addTraversability(Graph& graph, const std::unique_ptr<GraphingInput>& input);
};
}
