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
    for (const std::shared_ptr<Node> proposed : graph->getRegionNodes()) {
        // skip the node if its an island... its not traversable
        if(proposed->getConnectedIDs().empty()) continue;
        double closest_dist = std::numeric_limits<double>::max();
        std::shared_ptr<Node> closest_node;
        // find the closest node in the graph
        for (const std::shared_ptr<Node> existing : scene_graph_->getRegionNodes()) {
            if (existing && proposed) {
                double dist = calculateDistance(proposed->getUtmCoordinate(), existing->getUtmCoordinate());
                if(dist < closest_dist) {
                    closest_dist = dist;
                    closest_node = existing;
                }
            }
        }
        // If the closest nodes is moe than X m away, 
        // add the nodes to the graph and connect it to the closest 
        // region node.
        // TODO Make this a paramater
        if (closest_node && closest_dist > 5.0) {
            LOG(INFO) << "[SCENIC] Adding Region Node: " << proposed->getNodeID();
            scene_graph_->addNode(proposed);
            scene_graph_->addEdge(proposed, closest_node);
            scene_graph_->addEdge(closest_node, proposed);
            // TODO score this edge somehow
        }
    }
}

void StitchingProcessor::regionRegistrationViaBackProjection(const cv::Mat& coords, const std::shared_ptr<Graph>& graph, const GraphingInput& imagery)
{
    //std::vector<cv::Mat> channels;
    //cv::split(coords, channels);

    //double min_easting, max_easting;
    //cv::minMaxLoc(channels[0], &min_easting, &max_easting);

    //double min_northing, max_northing;
    //cv::minMaxLoc(channels[1], &min_northing, &max_northing);

    //std::vector<cv::Point> back_proped_pixels;
    //for (const std::shared_ptr<Node> existing : scene_graph_->getRegionNodes()) {
    //    UTMPoint utm = existing->getUtmCoordinate();
    //    if (utm.easting >= min_easting && utm.easting <= max_easting && utm.northing >= min_northing && utm.northing <= max_northing) {
    //        BufferSearchCoordinates searcher(coords, utm);
    //        std::pair<int, int> c = searcher.search(); 
    //        cv::Point pixel(c.first, c.second);

    //        back_proped_pixels.push_back(pixel);
    //    }
    //}
    //
    //std::vector<double> distances;
    //if (back_proped_pixels.size() > 1) {
    //    for (int i = 0; i < back_proped_pixels.size() - 1; i++) {
    //        double dist = cv::norm(back_proped_pixels[i], back_proped_pixels[i+1]);
    //        distances.push_back(dist);
    //    }
    //    double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
    //    double avg = sum / distances.size();
    //} else {
    //    // Maybe we can handle this with proposed nodes;
    //    LOG(WARNING) << "[SCENIC] You may be cooked here captain";
    //}

}

void StitchingProcessor::checkObjectNodes(const std::shared_ptr<Graph>& graph)
{
    //for (const std::shared_ptr<Node> existing : scene_graph_->getObjectNodes()) {
    //    double closest_dist = std::numeric_limits<double>::max();
    //    std::shared_ptr<Node> closest_node;
    //    for (const std::shared_ptr<Node> proposed : graph->getObjectNodes()) {
    //        // if distance is greater than n meters add the object node
    //        // TODO make this a parameter
    //        double distance = calculateDistance(proposed->getUtmCoordinate(), existing->getUtmCoordinate());
    //        if (distance < closest_dist) {
    //            closest_dist = distance;
    //            closest_node = proposed;
    //        }
    //    }
    //    if (closest_dist > 10.0 && !scene_graph_->contains(closest_node->getNodeID())) {
    //        LOG(INFO) << "[SCENIC] Adding Object Node To Scene Graph: " << closest_node->getNodeID();
    //        scene_graph_->addNode(closest_node);
    //    }
    //}
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
                LOG(INFO) << "[SCENIC] Initializing Scene Graph";
                scene_graph_ = raw_input->graph;
                localizeNodes(scene_graph_, raw_input->odom.getPose<Eigen::Isometry3d>());
            } else {
                // localize points in utm frame
                cv::Mat coords = cv::Mat::zeros(384, 512, CV_64FC2);
                cv::Mat K = rectifier_.getIntrinsics<cv::Mat>();
                cv::Mat D = rectifier_.getDistortion<cv::Mat>();

                std::shared_ptr<Graph> graph = raw_input->graph;
                Eigen::Isometry3d pose = raw_input->odom.getPose<Eigen::Isometry3d>();
                
                BufferLocalization localize_image(coords, K, D, pose);
                int r = coords.rows;
                int c = coords.cols;
                tbb::parallel_reduce(tbb::blocked_range2d<int>(0, r, 0, c), localize_image);

                localizeNodes(graph, pose);
                checkRegionNodes(graph);
                checkObjectNodes(graph);
                scene_graph_->setProcessID(graph->getProcessID());
            if (scene_graph_) outputCallback(scene_graph_);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
