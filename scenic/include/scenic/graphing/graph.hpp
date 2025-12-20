/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About the main graph object
*/

#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>

#include "scenic/graphing/edge.hpp"
#include "scenic/graphing/node.hpp"
#include "scenic/graphing/kmeans.hpp"
#include "scenic/core/processor_inputs.hpp"

namespace Scenic
{
class ScenicGraph
{

};

class UIDGenerator
{
    private:
        inline static std::atomic<uint64_t> counter{0};
    public:
        static uint64_t getNextUID()
        {
            return counter.fetch_add(1, std::memory_order_relaxed);
        }
};

class Graph
{
    public:
        Graph() = default;
        
        static Graph RegionAnalysis(const GraphingInput& input);
        static Graph ObjectAnalysis(const GraphingInput& input);
   
        void setNodes(const AdjacencyOutput& adj, std::map<uchar,cv::Point>& centroids, const int& cls_label);
        void setNodes(const std::vector<cv::Point>& centroids, const int& cls_label); 
        // get a node
        std::shared_ptr<Node> operator[](uint64_t nid);
        // get an edge
        //Edge operator[](const uint64_t& nid1, const uint64_t nid2) const;

        std::shared_ptr<Node> getNode(uint64_t nid);
        //Edge getEdge(const uint64_t& nid1, const uint64_t nid2) const;

        std::map<uint64_t, std::shared_ptr<Node>> getNodes() const;

    private:
        std::map<uint64_t, std::shared_ptr<Node>> nodes_; 
};
}
