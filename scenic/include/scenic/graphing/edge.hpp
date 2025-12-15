/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About track information on the edge
*/

#pragma once

#include "scenic/graphing/node.hpp"

namespace Scenic
{
class Edge
{
    public:
        Edge() = default;
        Edge(const Node node);
        Edge(const Node node, const float score);

        void addScore(const float score);

    private:
        Node connected_node_;

        float score_{-1};
};
} // namespace Scenic
