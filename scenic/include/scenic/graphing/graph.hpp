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

        static cv::Mat DrawGraph(Graph& graph, const cv::Mat& image);
        static cv::Mat DrawGraph(const std::shared_ptr<Graph> graph, const cv::Mat& image);
        friend Graph operator+(const RegionGraph& rg, const ObjectGraph& og);

        // get a node
        std::shared_ptr<Node> operator[](uint64_t nid);
        std::shared_ptr<Node> getNode(uint64_t nid);
        // get an edge
        std::shared_ptr<Edge> operator()(uint64_t nid1, uint64_t nid2);
        std::shared_ptr<Edge> getEdge(uint64_t nid1, uint64_t nid2);
        
        std::map<uint64_t, std::shared_ptr<Node>> getNodes() const;
        std::map<std::pair<uint64_t, uint64_t>, std::shared_ptr<Edge>> getEdges() const;
        std::vector<std::shared_ptr<Node>> getRegionNodes() const;
        std::vector<std::shared_ptr<Node>> getObjectNodes() const;

        void setEdgeScore(uint64_t nid1, uint64_t nid2, float score);
        
        void addNode(std::shared_ptr<Node> node);
        void addEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2);    

        bool isEmpty() const;
        void setEmptyStatus(bool b); 
        
        int getProcessID() const; 
        void setProcessID(int p);

        bool contains(uint64_t nid) const;

        void pruneEdge(std::shared_ptr<Edge> edge);
        void updateObjectEdge(std::shared_ptr<Node> region, std::shared_ptr<Node> object);

    protected:
        void initEdges();

        std::map<uint64_t, std::shared_ptr<Node>> nodes_;
        std::map<std::pair<uint64_t, uint64_t>, std::shared_ptr<Edge>> edges_;
        
        bool empty_{true};
        int pid_;
};

class RegionGraph : public Graph
{
    public:
        RegionGraph() = default;
        
        static RegionGraph RegionAnalysis(const GraphingInput& input, KMeans& kmeans);

        void setNodes(const AdjacencyOutput& adj, std::map<uchar,cv::Point>& centroids, std::unordered_map<uchar,size_t>& cls_label);
};

class ObjectGraph : public Graph
{
    public:
        ObjectGraph() = default;
        
        static ObjectGraph ObjectAnalysis(const GraphingInput& input);
        
        void setNodes(const std::vector<cv::Point>& centroids, const size_t& cls_label);   
};
}
