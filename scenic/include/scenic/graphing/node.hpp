/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About store the information of
* a single node in the graph
*/

#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "scenic/utils/geostructs.hpp"

namespace Scenic
{
class Node
{
    public:
        Node() = default;
        Node(uint64_t id, int cls_label, cv::Point pixel);

        void addConnection(const std::shared_ptr<Node> n);    
        
        std::vector<std::shared_ptr<Node>> getConnectedNodes() const;
        std::vector<uint64_t> getConnectedIDs() const;

        cv::Point getPixelCoordinate() const;
        uint64_t getNodeID() const;
        int getClassLabel() const;

    private:
        UTMPoint utm_;
        LatLonPoint latlon_;
        cv::Point pixel_;

        int label_;
        uint64_t nid_;
        std::vector<std::shared_ptr<Node>> connections_;
        std::vector<uint64_t> connection_ids_;
};
}

