/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About the main graph object
*/

#pragma once

#include "scenic/graphing/edge.hpp"
#include "scenic/graphing/node.hpp"
#include "scenic/graphing/kmeans.hpp"

namespace Scenic
{
class ScenicGraph
{

};

class Graph
{
    public:
        Graph() = default;
        
        static Graph SingleImageAnalysis(const GraphingInput& input);

    private:
        std::vector<std::pair<Node,Node>> nodes_; 
};
}
