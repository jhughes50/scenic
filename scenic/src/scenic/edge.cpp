/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About store edge information
*/

#include "scenic/graphing/edge.hpp"

using namespace Scenic;

Edge::Edge(const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2)
{
    edge_ = std::make_pair(n1, n2);
}

float Edge::getScore() const
{
    return score_;
}

void Edge::setScore(float score)
{
    score_ = score;
}   

std::shared_ptr<Node> Edge::getNode(const uint64_t& uid) const
{
    if (uid == edge_.first->getNodeID()) {
        return edge_.first;
    }
    else if (uid == edge_.second->getNodeID()) {
        return edge_.second;
    }
    return nullptr;
}

std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> Edge::getNodePair() const
{
    return edge_;
}
