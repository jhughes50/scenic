/*!
* @Author Jason Hughes
* @Date January 2025
*
* @About stitch the graphs together
*/

#include "scenic/core/stitching_processor.hpp"
#include <limits>
#include <unordered_map>

using namespace Scenic;

StitchingProcessor::StitchingProcessor(size_t capacity, const std::string& rect_path) : ThreadedProcessor(capacity)
{
    rectifier_ = Rectifier::Load(rect_path, 4);
}

void StitchingProcessor::setCallback(std::function<void(std::shared_ptr<Graph>)> callback)
{
    outputCallback = callback;
}

void StitchingProcessor::localizeNodes(std::shared_ptr<Graph>& graph, const Eigen::Isometry3d& pose)
{
    cv::Mat K = rectifier_.getIntrinsics<cv::Mat>();
    cv::Mat D = rectifier_.getDistortion<cv::Mat>();
    for (auto& [key, node] : graph->getNodes()) {
        cv::Point px = node->getPixelCoordinate();
        cv::Point2d pixel(static_cast<double>(px.x), static_cast<double>(px.y));
        std::vector<cv::Point2d> pixel_vec = {pixel};
        
        std::vector<cv::Point2d> undistorted;
        cv::undistortPoints(pixel_vec, undistorted, K, D);

        Eigen::Vector3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
        Eigen::Vector3d ray_imu = transforms_.T_imu_cam.linear() * ray_cam;
        Eigen::Vector3d ray_world = pose.linear() * ray_imu;

        double scale = -pose.translation().z() / ray_world(2);

        ray_imu = scale * ray_imu;
        Eigen::Vector3d pixel_imu = transforms_.T_imu_cam.translation() + ray_imu;
        Eigen::Vector3d pixel_world = pose.translation() + (pose.linear() * pixel_imu);
       
        node->setUtmCoordinate(pixel_world(0), pixel_world(1));
        //Glider::geodetics::UTMtoLL(pixel_world(1), pixel_world(0), '18s'
        // TODO set LatLon Point
    }
}

void StitchingProcessor::checkRegionNodes(const std::shared_ptr<Graph>& graph)
{
    std::unordered_map<uint64_t, std::pair<uint64_t, double>> node_pairs;
    for (const std::shared_ptr<Node>& proposed : graph->getRegionNodes()) {
        double closest_dist = std::numeric_limits<double>::max();
        std::shared_ptr<Node> closest_node;
        for (const std::shared_ptr<Node>& existing : scene_graph_->getRegionNodes()) {
            double distance = calculateDistance(proposed->getUtmCoordinate(), existing->getUtmCoordinate());
            if (distance < closest_dist) {
                closest_dist = distance;
                closest_node = existing;
            }
        }
        node_pairs[proposed->getNodeID()] = std::make_pair(closest_node->getNodeID(), closest_dist);
    }

    // break this up to ensure new nodes are connected to old nodes
    // add the edges
    for (const auto& [pid, eidd] : node_pairs) {
        if (eidd.second > 10.0) {
            LOG(INFO) << "[SCENIC] Adding Region Node To Scene Graph: " << pid; 
            scene_graph_->addNode(graph->getNode(pid));
            for (uint64_t nid : graph->getNode(pid)->getConnectedIDs()) {
                std::pair<uint64_t, double> existing = node_pairs[nid];
                scene_graph_->addEdge(scene_graph_->getNode(pid), scene_graph_->getNode(existing.first));
                // TODO how do I score this edge
            }
        }
    }
}

void StitchingProcessor::checkObjectNodes(const std::shared_ptr<Graph>& graph)
{
    for (const std::shared_ptr<Node> proposed : graph->getObjectNodes()) {
        for (const std::shared_ptr<Node> existing : scene_graph_->getObjectNodes()) {
            // if distance is greater than n meters add the object node
            // TODO make this a parameter
            if (calculateDistance(proposed->getUtmCoordinate(), existing->getUtmCoordinate()) > 5.0) {
                LOG(INFO) << "[SCENIC] Adding Object Node To Scene Graph: " << proposed->getNodeID();
                scene_graph_->addNode(proposed); 
            }
        }
    }
}

double StitchingProcessor::calculateDistance(const UTMPoint& p1, const UTMPoint& p2)
{
    return std::sqrt(std::pow(p2.easting-p1.easting, 2) + std::pow(p2.northing-p1.northing, 2));
}

void StitchingProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<GraphWithPose> raw_input = pop(Access::PRELOCK);
            if (!scene_graph_) {
                // graph is not initialized so we do that
                // this should only happend the first entry,
                // or if we need to reset for some reason
                scene_graph_ = raw_input->graph;
                localizeNodes(scene_graph_, raw_input->odom.getPose<Eigen::Isometry3d>());
            } else {
                // localize points in utm frame
                std::shared_ptr<Graph> graph = raw_input->graph;
                Eigen::Isometry3d pose = raw_input->odom.getPose<Eigen::Isometry3d>();
                localizeNodes(graph, pose);
                checkRegionNodes(graph);
                checkObjectNodes(graph);
                scene_graph_->setProcessID(graph->getProcessID());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (scene_graph_) outputCallback(scene_graph_);
    }
}
