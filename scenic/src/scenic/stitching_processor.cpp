/*!
* @Author Jason Hughes
* @Date January 2025
*
* @About stitch the graphs together
*/

#include "scenic/core/stitching_processor.hpp"
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_map>

using namespace Scenic;

StitchingProcessor::StitchingProcessor(size_t capacity, const std::string& rect_path, std::shared_ptr<Glider::Glider>& glider) : ThreadedProcessor(capacity)
{
    rectifier_ = Rectifier::Load(rect_path, 4);
    glider_ = glider;
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

void StitchingProcessor::localizeNode(std::shared_ptr<Node>& node, const Eigen::Isometry3d& pose)
{ 
    cv::Mat K = rectifier_.getIntrinsics<cv::Mat>();
    cv::Mat D = rectifier_.getDistortion<cv::Mat>();
    
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


void StitchingProcessor::addNodeByMax(const GraphWithPose& imagery)
{
    GraphingInput input = imagery.analysis;

    size_t input_size = input.getSize();
    for (size_t i = 0; i < input_size; ++i) {
        if (input.map[i].level == GraphLevel::REGION) {
            cv::Mat mask = input.map[i].mask;

            cv::Mat dist;
            cv::distanceTransform(mask, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);

            // Find the point with maximum distance
            double maxVal;
            cv::Point maxLoc;
            cv::minMaxLoc(dist, nullptr, &maxVal, nullptr, &maxLoc);

            uint64_t uid = UIDGenerator::getNextUID();
            std::shared_ptr<Node> node = std::make_shared<Node>(uid, input.map[i].uid, GraphLevel::REGION, maxLoc);

            localizeNode(node, input.odom.getPose<Eigen::Isometry3d>());
            scene_graph_->addNode(node);
        }
    }
}

void StitchingProcessor::checkObjectNodes(const std::shared_ptr<Graph>& graph, const Glider::Odometry& odom)
{
    Eigen::Isometry3d pose = odom.getPose<Eigen::Isometry3d>();
    if (!graph || !scene_graph_) return;

    for (std::shared_ptr<Node> proposed : graph->getObjectNodes()) {
        double closest_dist = std::numeric_limits<double>::max();
        std::shared_ptr<Node> closest_node;
        localizeNode(proposed, pose);
        for (std::shared_ptr<Node> existing : scene_graph_->getObjectNodes()) {
            double distance = calculateDistance(proposed->getUtmCoordinate(), existing->getUtmCoordinate());
            if (distance < closest_dist) {
                closest_dist = distance;
                closest_node = existing;
            }
        }

        if (closest_dist < 8.0) {
            // we've likely seen this object before
            uint64_t nid = closest_node->getNodeID();
            UTMPoint utm = proposed->getUtmCoordinate();
            double fx = rectifier_.getFocalLength();
            cv::Point px = proposed->getPixelCoordinate();
            Eigen::Vector2d center(256, 192); // hard code for now TODO fix this later 
            Eigen::Vector2d cam(px.x, px.y);
            Eigen::Vector2d global(utm.easting, utm.northing);
            int64_t stamp = odom.getTimestamp();
            glider_->addLandmark(stamp, nid, odom, global, cam, center, fx);
            Glider::PointWithCovariance filtered_pos = glider_->getLandmark(nid);
            
            closest_node->setUtmCoordinate(filtered_pos.x(), filtered_pos.y());
        } else if (closest_node) {
            uint64_t nid = proposed->getNodeID();
            UTMPoint utm = proposed->getUtmCoordinate();
            double fx = rectifier_.getFocalLength();
            cv::Point px = proposed->getPixelCoordinate();
            Eigen::Vector2d center(256, 192);
            Eigen::Vector2d cam(px.x, px.y);
            Eigen::Vector2d global(utm.easting, utm.northing);
            int64_t stamp = odom.getTimestamp();
            
            glider_->addLandmark(stamp, nid, odom, global, cam, center, fx);
            Glider::PointWithCovariance filtered_pos = glider_->getLandmark(nid);
            
            proposed->setUtmCoordinate(filtered_pos.x(), filtered_pos.y());
            
            scene_graph_->addNode(proposed);
        }
    }

    // object edge analysis
    for (std::shared_ptr<Node> object : scene_graph_->getObjectNodes()) {
        double closest_dist = std::numeric_limits<double>::max();
        std::shared_ptr<Node> closest_region;
        if (object) {
            for (std::shared_ptr<Node> region : scene_graph_->getRegionNodes()) {
                double dist = calculateDistance(object->getUtmCoordinate(), region->getUtmCoordinate());
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_region = region;
                }
            }
            if (closest_region) {
                if (!object->isConnected()) {
                    scene_graph_->addEdge(closest_region, object);
                } else { 
                    scene_graph_->updateObjectEdge(closest_region, object);
                }
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

                //regionRegistrationViaBackProjection(coords, *raw_input);  
                addNodeByMax(*raw_input);
                checkObjectNodes(graph, raw_input->odom);
                scene_graph_->setProcessID(graph->getProcessID());
            if (scene_graph_) outputCallback(scene_graph_);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
