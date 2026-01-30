/*!
* @Author Jason Hughes
* @Date Decemeber 2025
*
* @About orchestrate all the threads
*/

#include <csignal>
#include <glog/logging.h>
#include "scenic/core/scenic.hpp"
#define LOG(severity) google::LogMessage(__FILE__, __LINE__, google::GLOG_##severity).stream()

namespace Scenic
{

Scenic::Scenic(size_t capacity, const std::string& model_path, const std::string& config_path)
{
    pid_img_map_.setCapacity(capacity);
    pid_to_map_.setCapacity(capacity);
    pid_gi_map_.setCapacity(capacity);

    std::string params_path = "/usr/local/share/scenic/config/";
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

    glider_ = std::make_shared<Glider::Glider>(params_path+"glider-params.yaml");

    seg_processor_ = std::make_unique<SegmentationProcessor>(capacity, params_path, model_path, config_path);
    seg_processor_->setCallback([this](std::shared_ptr<GraphingInput> so) { 
        this->segmentationCallback(so); 
    });

    graph_processor_ = std::make_unique<GraphingProcessor>(capacity, params_path+"blackfly-5mm.yaml");
    graph_processor_->setCallback([this](std::shared_ptr<GraphWithPose> go) {
        this->imageGraphCallback(go);
    });

    stitching_processor_ = std::make_unique<StitchingProcessor>(capacity, params_path+"blackfly-5mm.yaml", glider_);
    stitching_processor_->setCallback([this](std::shared_ptr<Graph> go) {
        this->graphCallback(go);
    });

    tracking_processor_ = std::make_unique<TrackingProcessor>(capacity, params_path+"blackfly-5mm.yaml", params_path+"sticky-params.yaml");
    tracking_processor_->setCallback([this](std::shared_ptr<homography::HomographyResult> to) {
        this->trackingCallback(to);
    });

    FLAGS_minloglevel = 0;
    FLAGS_logtostderr = 1;
}

Scenic::~Scenic()
{
    stop();
}

void Scenic::start()
{
    LOG(INFO) << "[SCENIC] Starting Scenic";
    //tracking_processor_->start();
    seg_processor_->start();
    graph_processor_->start();
    stitching_processor_->start();
}

void Scenic::stop()
{
    LOG(INFO) << "[SCENIC] Stopping Scenic";
    //tracking_processor_->stop();
    seg_processor_->stop();
    graph_processor_->stop();
    stitching_processor_->stop();
}   

void Scenic::setText(std::vector<Text> text)
{
    for (const Text& t : text) {
        LOG(INFO) << "[SCENIC] Setting Class: " << t.label << " with uid: " << t.uid;
    }
    initialized_ = true;
    texts_.text = text;
}

std::shared_ptr<Graph> Scenic::getGraph()
{
    new_graph_ = false;
    return graph_;
}

void Scenic::push(int64_t timestamp, const cv::Mat& img, Glider::Odometry& odom)
{
    // create a pointer to input
    int pid = seg_processor_->generateProcessID();
    LOG(INFO) << "[SCENIC] Starting PID: " << pid;
    if (texts_.text.size() > 0 && current_state_.isInitialized()) {
        //pid_img_map_.insert(pid, img.clone());
        odom.setAltitude(35.0);
        std::unique_lock<std::mutex> lock(img_proc_mutex_);
        pid_status_map_[pid] = false;
        lock.unlock();
        SegmentationInput seg_model_input(pid, img, odom, texts_);
        seg_processor_->push(seg_model_input);
    }
    else {
        LOG(FATAL) << "[SCENIC] Trying to inference before text was set";
    }
    if (tracking_counter_ == 0) {
        prev_image_stamped_.image = img;
        prev_image_stamped_.stampd = 0.0;
        prev_image_stamped_.stampi = 0;
        prev_image_stamped_.odom = odom;
    } else {
        ImageStamped curr_image_stamped{img, odom, 0, 0};
        TrackingInput input{pid, prev_image_stamped_, curr_image_stamped};
        tracking_processor_->push(input);
        prev_image_stamped_ = curr_image_stamped;
    }
    tracking_counter_++;
}

void Scenic::addImage(double vo_ts, int64_t gt_ts, const cv::Mat& img, bool segment)
{
    // PASS
}

Glider::Odometry Scenic::addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    glider_->addImu(timestamp, accel, gyro, quat);
    Glider::Odometry odom;
    if (current_state_.isInitialized()) {
        odom = glider_->interpolate(timestamp);
    }
    return odom;
}

Glider::OdometryWithCovariance Scenic::addGPS(int64_t timestamp, Eigen::Vector3d& gps)
{
    glider_->addGps(timestamp, gps);
    current_state_ = glider_->optimize(timestamp);
    return current_state_;
}

Glider::Odometry Scenic::getStateEstimate(int64_t timestamp)
{
    Glider::Odometry odom;
    if (current_state_.isInitialized()) {
        odom = glider_->interpolate(timestamp);
    }
    return odom;
}

void Scenic::trackingCallback(std::shared_ptr<homography::HomographyResult> to)
{
    if (to) {
        LOG(INFO) << "[SCENIC] [TRACKING] Reprojection Error: " << to->reprojection_error;
    } else {
        LOG(INFO) << "[SCENIC] Waiting for Tracking to initialize";
    }
}

void Scenic::segmentationCallback(std::shared_ptr<GraphingInput> so)
{
    // get the size in an unsafe manner, why? tbd
    LOG(INFO) << "[SCENIC] [SEGMENTATION] Queue Size: " << seg_processor_->size(Access::UNLOCK);
    pid_img_map_.insert(so->pid, std::make_pair(so->image, so->map[0].mask));
    // BELOW is for tracking
    //std::lock_guard<std::mutex> lock(img_proc_mutex_);
    //if (pid_status_map_[so->pid]) {
    //    // get the tracking output for this pid and
    //    // pass to the graph constructor.
    //    std::shared_ptr<TrackingOutput> to = pid_to_map_.get(so->pid);
    //    LOG(INFO) << "[SCENIC] Got TrackingOutput first for " << so->pid;
    //} else {
    //    pid_status_map_[so->pid] = true;
    //    pid_gi_map_.insert(so->pid, so);
    //}
    graph_processor_->push(*so);
}

void Scenic::imageGraphCallback(std::shared_ptr<GraphWithPose> go)
{
    LOG(INFO) << "[SCENIC] [GRAPHING] Queue Size: " << graph_processor_->size(Access::UNLOCK);
    image_graph_ = go->graph;
    stitching_processor_->push(*go);
}

void Scenic::graphCallback(std::shared_ptr<Graph> go)
{
    LOG(INFO) << "[SCENIC] [STITCHING] Queue Size: " << stitching_processor_->size(Access::UNLOCK);
    LOG(INFO) << "[SCENIC] Finished PID: " << go->getProcessID();
    // do something
    graph_ = go;
    new_graph_ = true;
}

bool Scenic::isInitialized() const
{
    return initialized_;
}

bool Scenic::isNewGraph() const
{
    return new_graph_;
}

Glider::Odometry Scenic::getVisualOdometry() const
{
    return visual_odom_;
}

cv::Mat Scenic::getGraphImage() const
{
    cv::Mat display;
    if (image_graph_) {
        cv::Mat img = getMaskOverlay();
        display = Graph::DrawGraph(image_graph_, img);
    }
    return display;
}

cv::Mat Scenic::getMaskOverlay() const
{
    int pid = graph_->getProcessID();
    std::pair<cv::Mat, cv::Mat> images = pid_img_map_.get(pid);

    cv::Mat img = images.first;
    cv::Mat mask = images.second;

    cv::Mat colored_mask;
    cv::cvtColor(mask, colored_mask, cv::COLOR_GRAY2BGR);
    colored_mask.setTo(cv::Scalar(255, 127, 41), mask);  // green

    cv::Mat result;
    cv::addWeighted(img, 1.0, colored_mask, 0.3, 0, result);

    return result;
}

void Scenic::setOrigin(double easting, double northing)
{
    origin_ = UTMPoint{easting, northing};
}

template <typename T>
T Scenic::exportToJsonFormat(const Graph& graph)
{
    Json::Value root;
    
    Json::Value origin(Json::arrayValue);
    origin.append(origin_.easting);
    origin.append(origin_.northing);

    // export to the nodes
    Json::Value regions(Json::arrayValue);
    Json::Value objects(Json::arrayValue);
    for (const auto& [nid, node] : graph.getNodes()) {
        Json::Value entry;
        // make the label
        std::string label = texts_[node->getClassLabel()];
        std::replace(label.begin(), label.end(), ' ', '_');
        label += "_"+std::to_string(nid);
        entry["name"] = label;

        Json::Value coords(Json::arrayValue);
        UTMPoint point = node->getUtmCoordinate();
        coords.append(point.easting);
        coords.append(point.northing);
        entry["coords"] = coords;

        if (node->getNodeLevel() == GraphLevel::REGION) {
            regions.append(entry);
        } else if (node->getNodeLevel() == GraphLevel::OBJECT) {
            objects.append(entry);
        } else {
            LOG(ERROR) << "A node with no GraphLevel was found during export, NID: " << nid;
        }
    }
    root["objects"] = objects;
    root["regions"] = regions;

    Json::Value object_connections(Json::arrayValue);
    Json::Value region_connections(Json::arrayValue);
    for (const auto& [nids, edge] : graph.getEdges()) {
        std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> nodes = edge->getNodePair();

        std::string parent_label = texts_[nodes.first->getClassLabel()];
        parent_label += "_"+std::to_string(nids.first);

        std::string child_label = texts_[nodes.second->getClassLabel()];
        std::replace(child_label.begin(), child_label.end(), ' ', '_');
        child_label += "_"+std::to_string(nids.second);
        
        Json::Value names(Json::arrayValue);
        names.append(parent_label);
        names.append(child_label);

        Json::Value entry;
        entry["names"] = names;
        entry["score"] = edge->getScore();

        if (nodes.first->getNodeLevel() == GraphLevel::REGION && nodes.second->getNodeLevel() == GraphLevel::REGION) {
            region_connections.append(entry); 
        } else if (nodes.first->getNodeLevel() == GraphLevel::OBJECT || nodes.second->getNodeLevel() == GraphLevel::OBJECT) {
            object_connections.append(entry);
        }
    }
    root["object_connections"] = object_connections;
    root["region_connections"] = region_connections;

    if constexpr (std::is_same_v<Json::Value, T>) {
        return root;
    } else if constexpr (std::is_same_v<std::string, T>) {
        Json::FastWriter writer;
        std::string jsons = writer.write(root);
        return jsons;
    }
}

template Json::Value Scenic::exportToJsonFormat<Json::Value>(const Graph& graph);
template std::string Scenic::exportToJsonFormat<std::string>(const Graph& graph);

}// namespace Scenic
