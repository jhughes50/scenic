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
class Node 
{
    public:
        Node() = default
        Node(const int uid);    

        int getUID() const;
        UTMPoint getUTM() const;
        LatLonPoint getLatLon() const;
        std::string getLabel() const;

    private:
        UTMPoint utm_;
        LatLonPoint latlon_;

        int uid_;
        std::string label_;
};

class RegionNode : public Node
{
    public: 
        using Node::Node;
        
        void addNeighbor(const Edge edge);
        void addObject(const Edge edge);

    private:
        std::vector<Edge> neighbors_;
        std::vecotr<Edge> objects_;
};

class ObjectNode : public Node
{
    public:
        using Node::Node

        void AddParentNode(RegionNode node);

    private:
        RegionNode parent_node_;
}

}

