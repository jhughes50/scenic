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

class RegionGraph;
class ObjectGraph;

class Graph
{
    public:
        Graph() = default;
        Graph(std::map<uint64_t, std::shared_ptr<Node>> nodes);
    
        friend Graph operator+(const RegionGraph& rg, const ObjectGraph& og);

        // get a node
        std::shared_ptr<Node> operator[](uint64_t nid);
        std::shared_ptr<Node> getNode(uint64_t nid);
        // get an edge
        std::shared_ptr<Edge> operator()(uint64_t nid1, uint64_t nid2);
        std::shared_ptr<Edge> getEdge(uint64_t nid1, uint64_t nid2);
        
        std::map<uint64_t, std::shared_ptr<Node>> getNodes() const;
        std::map<std::pair<uint64_t, uint64_t>, std::shared_ptr<Edge>> getEdges() const;
        std::map<uint64_t, std::shared_ptr<Node>> getRegionNodes() const;

        void setEdgeScore(uint64_t nid1, uint64_t nid2, float score);

    protected:
        void initEdges();

        std::map<uint64_t, std::shared_ptr<Node>> nodes_;
        std::map<std::pair<uint64_t, uint64_t>, std::shared_ptr<Edge>> edges_;
};

class RegionGraph : public Graph
{
    public:
        RegionGraph() = default;
        
        static RegionGraph RegionAnalysis(const GraphingInput& input);

        void setNodes(const AdjacencyOutput& adj, std::map<uchar,cv::Point>& centroids, const int& cls_label);
};

class ObjectGraph : public Graph
{
    public:
        ObjectGraph() = default;
        
        static ObjectGraph ObjectAnalysis(const GraphingInput& input);
        
        void setNodes(const std::vector<cv::Point>& centroids, const int& cls_label); 
};
}
