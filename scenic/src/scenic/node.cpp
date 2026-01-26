/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About stoe the information relative to each node
*/

#include "scenic/graphing/node.hpp"

using namespace Scenic;

Node::Node(uint64_t id, size_t cls_label, GraphLevel level, cv::Point pixel)
{
    nid_ = id;
    label_ = cls_label;
    pixel_ = pixel;
    level_ = level;
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

size_t Node::getClassLabel() const
{
    return label_;
}

GraphLevel Node::getNodeLevel() const
{
    return level_;
}

cv::Point Node::getPixelCoordinate() const
{
    return pixel_;
}

void Node::setPixelCoordinate(cv::Point p)
{
    pixel_ = p;
}

void Node::setUtmCoordinate(double easting, double northing)
{
    utm_.easting = easting;
    utm_.northing = northing;
}

UTMPoint Node::getUtmCoordinate() const
{
    return utm_;
}

void Node::setLatLonCoordinate(double lat, double lon)
{
    latlon_.latitude = lat;
    latlon_.longitude = lon;
}

LatLonPoint Node::getLatLonCoordinate() const
{
    return latlon_;
}

void Node::removeConnectedNode(std::shared_ptr<Node> node)
{
    std::vector<uint64_t>::iterator itid = std::find(connection_ids_.begin(), connection_ids_.end(), node->getNodeID());
    if (itid != connection_ids_.end()) {
        connection_ids_.erase(itid);
    }

    std::vector<std::shared_ptr<Node>>::iterator itn = std::find(connections_.begin(), connections_.end(), node);
    if (itn != connections_.end()) {
        connections_.erase(itn);
    }
}

bool Node::isConnected() const
{
    if (connection_ids_.size() > 0) {
        return true;
    } else {
        return false;
    }
}
