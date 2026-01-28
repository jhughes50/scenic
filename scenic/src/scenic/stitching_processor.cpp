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

void StitchingProcessor::regionRegistrationViaBackProjection(const cv::Mat& coords, const GraphWithPose& input)
{
    GraphingInput imagery = input.analysis;
    size_t input_size = imagery.getSize();
    // find max and min utm coords
    std::vector<cv::Mat> channels;
    cv::split(coords, channels);

    double min_easting, max_easting;
    cv::minMaxLoc(channels[0], &min_easting, &max_easting);

    double min_northing, max_northing;
    cv::minMaxLoc(channels[1], &min_northing, &max_northing);
    // back project existing nodes into the image
    std::vector<cv::Point2f> back_proj_pixels;
    std::vector<cv::Point> back_proj_ipixels;
    std::unordered_map<uint64_t, cv::Point> pixel_id_map;
    for (const std::shared_ptr<Node> existing : scene_graph_->getRegionNodes()) {
        UTMPoint utm = existing->getUtmCoordinate();
        if (utm.easting >= min_easting && utm.easting <= max_easting && utm.northing >= min_northing && utm.northing <= max_northing) {
            BufferSearchCoordinates searcher(coords, utm);
            std::pair<int, int> c = searcher.search(); 
            cv::Point ipixel(c.first, c.second);
            // update pixel coordinate to be in the current image
            existing->setPixelCoordinate(ipixel);
            cv::Point2f pixel(static_cast<float>(c.first), static_cast<float>(c.second));
            if (c.first == -1 || c.second == -1) continue;
            back_proj_pixels.push_back(pixel);
            pixel_id_map[existing->getNodeID()] = ipixel;
            back_proj_ipixels.push_back(ipixel);
        }
    }
   
    // cluster the new nodes with the old nodes;
    int region_count = 0;
    cv::Mat region_mask = cv::Mat::zeros(imagery.image.size(), CV_8UC1); 
    for (size_t i = 0; i < input_size; ++i) {
        if (imagery.map[i].level == GraphLevel::REGION) {
            cv::bitwise_or(region_mask, imagery.map[i].mask, region_mask);
            region_count++;
        }
    }
    // if there are no regions detected 
    if (region_count == 0) return;
    int k = 20;
    int new_k = std::max(0, k - static_cast<int>(back_proj_pixels.size())); 
    // if this is 0 we're not adding any new nodes... we can exit
    if (new_k == 0) return;
    
    KMeansOutput output = kmeans_.nFixedLloyds(region_mask, back_proj_pixels, new_k, 100, 1e-4);
    AdjacencyOutput adj = kmeans_.connectRegions(output.points, output.voronoi, k); 

    std::map<uchar, cv::Point> centers;
    std::unordered_map<uchar, size_t> labels;
    for (const cv::Point& p : output.centroids) {
        uchar region_id = output.voronoi.at<uchar>(p);
        centers[region_id] = p;
        for (size_t j = 0; j < input_size; ++j) { 
            if (imagery.map[j].mask.at<uchar>(p) == 1) {
                labels[region_id] = imagery.map[j].uid;
                break;
            }
        }
    }

    // add the new nodes to the graph
    std::map<uchar, uint64_t> uid_map;
    for (const auto& [key, vals] : adj.adjacency) {
        if (centers.find(key) != centers.end()) {
            uint64_t uid = UIDGenerator::getNextUID();
            uid_map[key] = uid;
            cv::Point pixel_coord = centers[key];
            size_t cls_label = labels[key];
            std::shared_ptr<Node> node = std::make_shared<Node>(uid, cls_label, GraphLevel::REGION, pixel_coord);
            localizeNode(node, input.odom.getPose<Eigen::Isometry3d>());
            
            scene_graph_->addNode(node);
        }
    }

    // add existing nodes to centers and uid map
    for (const auto& [nid, p] : pixel_id_map) {
        uchar region_id = output.voronoi.at<uchar>(p);
        centers[region_id] = p;
        uid_map[region_id] = nid;
    }

    // add the edges
    for (const auto& [key, vals] : adj.adjacency) {
        for (const uchar& v : vals) {
            if (uid_map.find(key) != uid_map.end() && uid_map.find(v) != uid_map.end()) {
                uint64_t parent_id = uid_map[key];
                uint64_t child_id = uid_map[v];

                std::shared_ptr<Node> parent_node = scene_graph_->getNode(parent_id);
                std::shared_ptr<Node> child_node = scene_graph_->getNode(child_id);
                scene_graph_->addEdge(parent_node, child_node);
                std::shared_ptr<Edge> new_edge = scene_graph_->getEdge(parent_id, child_id);
                bool result = Traversability::scoreEdge(new_edge, imagery);
                if (!result) scene_graph_->pruneEdge(new_edge);
            }
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
            Eigen::Vector3d filtered_pos = glider_->getLandmark(nid);
            
            closest_node->setUtmCoordinate(filtered_pos(0), filtered_pos(1));
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
            Eigen::Vector3d filtered_pos = glider_->getLandmark(nid);
            
            proposed->setUtmCoordinate(filtered_pos(0), filtered_pos(1));
            
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

                regionRegistrationViaBackProjection(coords, *raw_input);  
                checkObjectNodes(graph, raw_input->odom);
                scene_graph_->setProcessID(graph->getProcessID());
            if (scene_graph_) outputCallback(scene_graph_);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
