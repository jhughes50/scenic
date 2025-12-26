/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About graph scructure
*/

#include "scenic/graphing/graph.hpp"

using namespace Scenic;

Graph::Graph(std::map<uint64_t, std::shared_ptr<Node>> nodes)
{
    nodes_ = nodes;
}

std::shared_ptr<Node> Graph::operator[](uint64_t nid)
{
    return nodes_[nid];
}

std::shared_ptr<Node> Graph::getNode(uint64_t nid) 
{
    return nodes_[nid];
}

std::map<uint64_t, std::shared_ptr<Node>> Graph::getNodes() const
{
    return nodes_;
}

std::shared_ptr<Edge> Graph::operator()(uint64_t nid1, uint64_t nid2)
{
    std::shared_ptr<Node> node1 = nodes_[nid1];
    std::shared_ptr<Edge> edge = nullptr;
    for(const auto n : node1->getConnectedNodes()) {
        if (n->getNodeID() == nid2) {
            edge = std::make_shared<Edge>(node1, n);
            break;
        }
    }
    return edge;
}

Scenic::Graph Scenic::operator+(const RegionGraph& rg, const ObjectGraph& og)
{
    std::map<uint64_t, std::shared_ptr<Node>> region_nodes = rg.getNodes();
    std::map<uint64_t, std::shared_ptr<Node>> object_nodes = og.getNodes();
    
    std::map<uint64_t, std::shared_ptr<Node>> merged_nodes;
    for (const auto& [key, node] : region_nodes) {
        merged_nodes[key] = node;
    }
    for (const auto& [key, node] : object_nodes) {
        merged_nodes[key] = node;
    }
    Graph merged_graph(merged_nodes);

    for(const auto& [okey, on] : object_nodes) {
        std::shared_ptr<Node> closest_node;
        cv::Point op = on->getPixelCoordinate();
        double min_dist = std::numeric_limits<double>::max();
        for(const auto& [rkey, rn] : region_nodes) {
            cv::Point rp = rn->getPixelCoordinate();
            double dist = cv::norm(op - rp);
            if (dist < min_dist) {
                min_dist = dist;
                closest_node = rn;
            }
        }
        closest_node->addConnection(on);
        on->addConnection(closest_node);
    }
    
    return merged_graph;
}


RegionGraph RegionGraph::RegionAnalysis(const GraphingInput& input)
{
    size_t input_size = input.getSize();

    cv::Mat region_mask = cv::Mat::zeros(input.image.rows, input.image.cols, CV_8UC1);
    for (size_t i = 0; i < input_size; ++i) {
        if (input.texts.text[i].level == GraphLevel::REGION) {
            cv::bitwise_or(region_mask, input.masks[i], region_mask);
        }
    }

    //int k = getNumClusters(region_mask);
    int k = 6;
    KMeansOutput output = KMeans::Cluster(region_mask, k); 
    AdjacencyOutput graph = KMeans::ConnectRegions(output.points, output.voronoi, k);

    std::map<uchar, cv::Point> centers;
    for (const cv::Point p : output.centroids) {
        uchar region_id = output.voronoi.at<uchar>(p);
        centers[region_id] = p;
    }

    RegionGraph region_graph;
    region_graph.setNodes(graph, centers, 0);

    return region_graph;
}

void RegionGraph::setNodes(const AdjacencyOutput& adj, std::map<uchar, cv::Point>& centroids, const int& cls_label)
{
    std::map<uchar, uint64_t> uid_map;
    for (const auto& [key, vals] : adj.adjacency) {
        uint64_t uid = UIDGenerator::getNextUID();
        uid_map[key] = uid;
        cv::Point pixel_coord = centroids[key];
        std::shared_ptr<Node> n = std::make_shared<Node>(uid, cls_label, pixel_coord);
        nodes_[uid] = n;
    }

    for (const auto& [key, vals] : adj.adjacency) {
        for (const uchar& v : vals) {
            uint64_t parent_id = uid_map[key];
            uint64_t child_id = uid_map[v];
            nodes_[parent_id]->addConnection(nodes_[child_id]);
        }
    }
}

ObjectGraph ObjectGraph::ObjectAnalysis(const GraphingInput& input)
{
    size_t input_size = input.getSize();
    std::vector<cv::Point> cluster_centroids;
    for (size_t i = 0; i < input_size; ++i) {
        if (input.texts.text[i].level == GraphLevel::OBJECT) {
            cv::Mat labels, stats, centroids;
            int num_clusters = cv::connectedComponentsWithStats(input.masks[i],
                                                                labels,
                                                                stats,
                                                                centroids,
                                                                8,
                                                                CV_32S);



            for (int i = 1; i < num_clusters; i++) {
                cv::Point centroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
                cluster_centroids.push_back(centroid);
            }
        }
    }

    ObjectGraph graph;
    graph.setNodes(cluster_centroids, 1);
    return graph;
}



void ObjectGraph::setNodes(const std::vector<cv::Point>& centroids, const int& cls_label)
{
    for (const cv::Point& c : centroids) {
        uint64_t uid = UIDGenerator::getNextUID();
        std::shared_ptr<Node> n = std::make_shared<Node>(uid, cls_label, c);
        nodes_[uid] = n;
    }
}



