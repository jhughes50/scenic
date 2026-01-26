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
#include "scenic/core/processor_inputs.hpp"

namespace Scenic
{
class Node
{
    public:
        Node() = default;
        Node(uint64_t id, size_t cls_label, GraphLevel level, cv::Point pixel);

        void addConnection(const std::shared_ptr<Node> n);    
        
        std::vector<std::shared_ptr<Node>> getConnectedNodes() const;
        std::vector<uint64_t> getConnectedIDs() const;

        cv::Point getPixelCoordinate() const;
        uint64_t getNodeID() const;
        size_t getClassLabel() const;
        GraphLevel getNodeLevel() const;
        UTMPoint getUtmCoordinate() const;
        LatLonPoint getLatLonCoordinate() const;
        bool isConnected() const;

        void setUtmCoordinate(double easting, double northing);
        void setLatLonCoordinate(double lat, double lon);

        void removeConnectedNode(std::shared_ptr<Node> node);

    private:
        UTMPoint utm_;
        LatLonPoint latlon_;
        cv::Point pixel_;

        size_t label_;
        uint64_t nid_;
        GraphLevel level_;
        std::vector<std::shared_ptr<Node>> connections_;
        std::vector<uint64_t> connection_ids_;
};
}

