/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About stoe the information relative to each node
*/

#include "scenic/graphing/node.hpp"

using namespace Scenic;

Node::Node(uint64_t id, int cls_label, cv::Point pixel)
{
    nid_ = id;
    label_ = cls_label;
    pixel_ = pixel;
}

void Node::addConnection(const std::shared_ptr<Node> n)
{
    connections_.push_back(n);
    connection_ids_.push_back(n->getNodeID());
}

std::vector<std::shared_ptr<Node>> Node::getConnectedNodes() const
{
    return connections_;
}

std::vector<uint64_t> Node::getConnectedIDs() const
{
    return connection_ids_;
}  

uint64_t Node::getNodeID() const
{
    return nid_;
}

int Node::getClassLabel() const
{
    return label_;
}

cv::Point Node::getPixelCoordinate() const
{
    return pixel_;
}
