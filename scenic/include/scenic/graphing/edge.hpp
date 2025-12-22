/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About track information on the edge
*/

#pragma once

#include <memory>

#include "scenic/graphing/node.hpp"

namespace Scenic
{
class Edge
{
    public:
        Edge() = default;
        Edge(const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2);

        float getScore() const;
        void setScore(float score);

        std::shared_ptr<Node> getNode(const uint64_t& uid) const;
        std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> getNodePair() const;

    private:
        std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> edge_;
        float score_;
};
} // namespace Scenic
