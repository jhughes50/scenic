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

std::vector<std::shared_ptr<Node>> Graph::getRegionNodes() const
{
    std::vector<std::shared_ptr<Node>> region_nodes;
    for (const auto& [key, node] : nodes_) {
        if (node->getNodeLevel() == GraphLevel::REGION) {
            region_nodes.push_back(node);
        }
    }

    return region_nodes;
}

std::vector<std::shared_ptr<Node>> Graph::getObjectNodes() const
{
    std::vector<std::shared_ptr<Node>> object_nodes;
    for (const auto& [key, node] : nodes_) {
        if (node->getNodeLevel() == GraphLevel::OBJECT) {
            object_nodes.push_back(node);
        }
    }
    return object_nodes;
}

std::map<std::pair<uint64_t, uint64_t>, std::shared_ptr<Edge>> Graph::getEdges() const
{
    return edges_;
}

std::shared_ptr<Edge> Graph::operator()(uint64_t nid1, uint64_t nid2)
{
    return getEdge(nid1, nid2);
}

void Graph::addNode(std::shared_ptr<Node> node)
{
    uint64_t nid = node->getNodeID();
    nodes_[nid] = node;
}

std::shared_ptr<Edge> Graph::getEdge(uint64_t nid1, uint64_t nid2)
{
    std::pair<uint64_t, uint64_t> node_pair(nid1, nid2);
    std::shared_ptr<Edge> edge = nullptr;
    if (edges_.find(node_pair) != edges_.end()) {
        edge = edges_[node_pair];
    }

    return edge;
}

void Graph::addEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)
{
    if (nodes_.find(n1->getNodeID()) == nodes_.end()) {
        LOG(WARNING) << "[SCENIC] Trying to add edge with node that doesn't exist, adding node to graph";
        uint64_t nid = n1->getNodeID();
        nodes_[nid] = n1;
    }
    if (nodes_.find(n2->getNodeID()) == nodes_.end()) {
        LOG(WARNING) << "[SCENIC] Trying to add edge with node that doesn't exist, adding node to graph";
        uint64_t nid = n2->getNodeID();
        nodes_[nid] = n2; 
    }

    std::pair<uint64_t, uint64_t> idp(n1->getNodeID(), n2->getNodeID());
    // ensure the edge doesnt already exist
    if (edges_.find(idp) == edges_.end()) {
        std::shared_ptr<Edge> edge = std::make_shared<Edge>(n1, n2);
        edges_[idp] = edge;
    }
}

void Graph::initEdges()
{
    for (const auto& [key, n1] : nodes_) {
        for (const std::shared_ptr<Node> n2 : n1->getConnectedNodes()) {
            std::shared_ptr<Edge> edge = std::make_shared<Edge>(n1, n2);
            std::pair<uint64_t, uint64_t> node_pair(n1->getNodeID(), n2->getNodeID());
            edges_[node_pair] = edge;
        }
    }
}

void Graph::setEdgeScore(uint64_t nid1, uint64_t nid2, float score)
{
    std::shared_ptr<Edge> edge = getEdge(nid1, nid2);
    edge->setScore(score);
}

bool Graph::isEmpty() const
{
    return empty_;
}

void Graph::setEmptyStatus(bool b)
{
    empty_ = b;
}

int Graph::getProcessID() const
{
    return pid_;
}

void Graph::setProcessID(int p)
{
    pid_ = p;
}

bool Graph::contains(uint64_t nid) const
{
    if (nodes_.find(nid) != nodes_.end()) {
        return true;
    } else {
        return false;
    }
}

void Graph::pruneEdge(std::shared_ptr<Edge> edge)
{
    std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> pair = edge->getNodePair();
    std::pair<uint64_t, uint64_t> nids(pair.first->getNodeID(), pair.second->getNodeID());
    std::pair<uint64_t, uint64_t> rnids(pair.second->getNodeID(), pair.first->getNodeID());
    // erase by key if they are found
    if (edges_.find(nids) != edges_.end()) {
        edges_.erase(nids);
    }
    if (edges_.find(rnids) != edges_.end()) {
        edges_.erase(rnids);
    }
    pair.first->removeConnectedNode(pair.second);
    pair.second->removeConnectedNode(pair.first);
}

void Graph::updateObjectEdge(std::shared_ptr<Node> region, std::shared_ptr<Node> object)
{
    // prune any existing edge connected to the object node 
    for (const auto& [eids, edge] : edges_) {
        if (eids.first == object->getNodeID() || eids.second == object->getNodeID()) {
            pruneEdge(edge);
        }
    }
    // add connection to region as new edge
    addEdge(region, object);
}

cv::Mat Graph::DrawGraph(Graph& graph, const cv::Mat& image)
{
    cv::Mat display = image.clone();
    
    for (const auto& [key, node] : graph.getNodes()) {
        cv::Point k = graph[key]->getPixelCoordinate();
        if (node->getNodeLevel() == REGION) {
            cv::circle(display, k, 5, cv::Scalar(185, 128, 41), -1);
        } else if (node->getNodeLevel() == OBJECT) {
            cv::circle(display, k, 5, cv::Scalar(128, 185, 41), -1); 
        }
        for (const uint64_t& nid : node->getConnectedIDs()) {
            cv::Point p = graph[nid]->getPixelCoordinate();
            if (node->getNodeLevel() == REGION) {
                cv::line(display, k, p, cv::Scalar(185, 128, 41), 2);
            } else if (node->getNodeLevel() == OBJECT) {
                cv::line(display, k, p, cv::Scalar(128, 185, 41), 2);
            }
        }
    }

    return display;
}

cv::Mat Graph::DrawGraph(const std::shared_ptr<Graph> graph, const cv::Mat& image)
{
    cv::Mat display = image.clone();
    
    for (const auto& [key, node] : graph->getNodes()) {
        cv::Point k = graph->getNode(key)->getPixelCoordinate();
        if (node->getNodeLevel() == REGION) {
            cv::circle(display, k, 5, cv::Scalar(185, 128, 41), -1);
        } else if (node->getNodeLevel() == OBJECT) {
            cv::circle(display, k, 5, cv::Scalar(128, 185, 41), -1); 
        }
        for (const uint64_t& nid : node->getConnectedIDs()) {
            cv::Point p = graph->getNode(nid)->getPixelCoordinate();
            if (node->getNodeLevel() == REGION) {
                cv::line(display, k, p, cv::Scalar(185, 128, 41), 2);
            } else if (node->getNodeLevel() == OBJECT) {
                cv::line(display, k, p, cv::Scalar(128, 185, 41), 2);
            }
        }
    }

    return display;
}

Scenic::Graph Scenic::operator+(const RegionGraph& rg, const ObjectGraph& og)
{
    std::map<uint64_t, std::shared_ptr<Node>> region_nodes = rg.getNodes();
    std::map<uint64_t, std::shared_ptr<Node>> object_nodes = og.getNodes();
    
    std::map<uint64_t, std::shared_ptr<Node>> merged_nodes;
    for (const auto& [key, node] : region_nodes) {
        if (node) merged_nodes[key] = node;
    }
    for (const auto& [key, node] : object_nodes) {
        if (node) merged_nodes[key] = node;
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
        if (closest_node) {
            closest_node->addConnection(on);
            on->addConnection(closest_node);
        }
    }
    merged_graph.initEdges();

    return merged_graph;
}


RegionGraph RegionGraph::RegionAnalysis(const GraphingInput& input, KMeans& kmeans)
{   
    size_t input_size = input.getSize();
    RegionGraph region_graph;

    int region_count = 0;
    cv::Mat region_mask = cv::Mat::zeros(input.image.size(), CV_8UC1); 
    for (size_t i = 0; i < input_size; ++i) {
        if (input.map[i].level == GraphLevel::REGION) {
            cv::bitwise_or(region_mask, input.map[i].mask, region_mask);
            region_count++;
        }
    }
    // if there are no regions detected 
    if (region_count == 0) return region_graph;
   
    KMeansOutput output;
    AdjacencyOutput graph;
    try {
        int k = 8; //kmeans.getNumClusters(region_mask, 35);// input.odom.getAltitude());
        output = kmeans.cluster(region_mask, k); 
        graph = kmeans.connectRegions(output.points, output.voronoi, k);
    } catch (const cv::Exception& e) {
        // there was no region detected
        // so return and empty graph.
        return region_graph;
    }

    std::map<uchar, cv::Point> centers;
    std::unordered_map<uchar, size_t> labels;
    for (const cv::Point p : output.centroids) {
        uchar region_id = output.voronoi.at<uchar>(p);
        centers[region_id] = p;
        for (size_t i = 0; i < input_size; ++i) {
            if (input.map[i].mask.at<uchar>(p) == 1) {
                labels[region_id] = input.map[i].uid;
                break;
            }
        }
    }

    region_graph.setNodes(graph, centers, labels);

    return region_graph;
}

void RegionGraph::setNodes(const AdjacencyOutput& adj, std::map<uchar, cv::Point>& centroids, std::unordered_map<uchar,size_t>& labels)
{
    setEmptyStatus(false);
    std::map<uchar, uint64_t> uid_map;
    for (const auto& [key, vals] : adj.adjacency) {
        if (centroids.find(key) != centroids.end()) {
            uint64_t uid = UIDGenerator::getNextUID();
            uid_map[key] = uid;
            cv::Point pixel_coord = centroids[key];
            size_t cls_label = labels[key];
            std::shared_ptr<Node> n = std::make_shared<Node>(uid, cls_label, GraphLevel::REGION, pixel_coord);
            nodes_[uid] = n;
        }
    }

    for (const auto& [key, vals] : adj.adjacency) {
        for (const uchar& v : vals) {
            if (uid_map.find(key) != uid_map.end() and uid_map.find(v) != uid_map.end()) {
                uint64_t parent_id = uid_map[key];
                uint64_t child_id = uid_map[v];
                nodes_[parent_id]->addConnection(nodes_[child_id]);
            }
        }
    }
}

ObjectGraph ObjectGraph::ObjectAnalysis(const GraphingInput& input)
{
    ObjectGraph graph;
    size_t input_size = input.getSize();
    for (size_t i = 0; i < input_size; ++i) {
        if (input.map[i].level == GraphLevel::OBJECT) {
            std::vector<cv::Point> cluster_centroids;
            cv::Mat labels, stats, centroids;
            int num_clusters = cv::connectedComponentsWithStats(input.map[i].mask,
                                                                labels,
                                                                stats,
                                                                centroids,
                                                                8,
                                                                CV_32S);

            std::vector<bool> merged(num_clusters, false);
            double merge_threshold = 30.0;
            for (int j = 1; j < num_clusters; j++) {
                cv::Point centroid(centroids.at<double>(j, 0), centroids.at<double>(j, 1));
                
                // Check if this centroid is close to any existing cluster
                bool merged = false;
                for (int c = 0; c < cluster_centroids.size(); c++) {
                    double dist = cv::norm(centroid - cluster_centroids[c]);
                    if (dist < merge_threshold) {
                        // Average this centroid into the existing cluster
                        cluster_centroids[c].x = (cluster_centroids[c].x + centroid.x) / 2;
                        cluster_centroids[c].y = (cluster_centroids[c].y + centroid.y) / 2;
                        merged = true;
                        break;
                    }
                }
                
                // If not merged, add as new cluster
                if (!merged) {
                    cluster_centroids.push_back(centroid);
                }
            }
            graph.setNodes(cluster_centroids, input.map[i].uid);       
        }
    }
    return graph;
}



void ObjectGraph::setNodes(const std::vector<cv::Point>& centroids, const int& cls_label)
{
    setEmptyStatus(false);
    for (const cv::Point& c : centroids) {
        uint64_t uid = UIDGenerator::getNextUID();
        std::shared_ptr<Node> n = std::make_shared<Node>(uid, cls_label, GraphLevel::OBJECT, c);
        nodes_[uid] = n;
    }
}



