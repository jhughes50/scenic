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

ScenicOrchestrator::ScenicOrchestrator(size_t capacity, const std::string& model_path, const std::string& config_path)
{
    database_ = std::make_shared<ScenicDatabase>();

    std::string params_path = "/usr/local/share/scenic/config/";
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

    glider_ = std::make_shared<Glider::Glider>(params_path+"glider-params.yaml");

    seg_processor_ = std::make_unique<SegmentationProcessor>(capacity, params_path, model_path, config_path, database_);
    seg_processor_->setCallback([this](int pid) { 
        this->segmentationCallback(pid); 
    });

    //graph_processor_ = std::make_unique<GraphingProcessor>(capacity, params_path+"blackfly-5mm.yaml");
    //graph_processor_->setCallback([this](std::shared_ptr<GraphWithPose> go) {
    //    this->imageGraphCallback(go);
    //});

    stitching_processor_ = std::make_unique<StitchingProcessor>(capacity, params_path+"blackfly-5mm.yaml", glider_, database_);
    stitching_processor_->setCallback([this](std::shared_ptr<Graph> go) {
        this->graphCallback(go);
    });

    tracking_processor_ = std::make_unique<TrackingProcessor>(capacity, params_path+"blackfly-5mm.yaml", database_);
    tracking_processor_->setCallback([this](int pid) {
        this->trackingCallback(pid);
    });

    FLAGS_minloglevel = 0;
    FLAGS_logtostderr = 1;
}

ScenicOrchestrator::~ScenicOrchestrator()
{
    stop();
}

void ScenicOrchestrator::start()
{
    LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Starting";
    tracking_processor_->start();
    seg_processor_->start();
    //graph_processor_->start();
    stitching_processor_->start();
}

void ScenicOrchestrator::stop()
{
    LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Stopping";
    tracking_processor_->stop();
    seg_processor_->stop();
    //graph_processor_->stop();
    stitching_processor_->stop();
}   

void ScenicOrchestrator::setText(std::vector<Text> text)
{
    for (const Text& t : text) {
        database_->addCid(t.uid);
        database_->insert<ScenicType::Class>(t.uid, t.label);
        database_->insert<ScenicType::Level>(t.uid, t.level);
        database_->insert<ScenicType::Priority>(t.uid, t.priority);
        LOG(INFO) << "[SCENIC] [DATABASE] Setting Class: " << t.label << " with uid: " << t.uid;
    }
    initialized_ = true;
    texts_.text = text;
    LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Initialized";
}

std::shared_ptr<Graph> ScenicOrchestrator::getGraph()
{
    new_graph_ = false;
    return graph_;
}

void ScenicOrchestrator::push(int64_t timestamp, const cv::Mat& img, Glider::Odometry& odom)
{
    // create a pointer to input
    int pid = seg_processor_->generateProcessID();

    if (!tracking_initialized_ || !current_state_.isInitialized() || !initialized_) {
        prev_image_ = img;
        tracking_initialized_ = true;
    } else {
        LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Starting PID: " << pid;
        TrackingInput tracking_input{pid, prev_image_, img};
        SegmentationInput seg_input{pid, img};
            
        odom.setAltitude(35.0);

        database_->insert<ScenicType::Odometry>(pid, odom);
        database_->insert<ScenicType::Image>(pid, img);
        pid_stamped_[timestamp] = pid;

        tracking_processor_->push(tracking_input);
        seg_processor_->push(seg_input);

        prev_image_ = img;
    }

    // only track the most recent N frames;
    if (pid_stamped_.size() > 10) {

        std::map<int64_t, int>::iterator it = pid_stamped_.begin();
        int oldest_pid = it->second;
        LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Removing PID " << oldest_pid << " from database";
        database_->eraseAll(oldest_pid);
        pid_stamped_.erase(it);
    }
    // For debug only, this takes up a lot 
    // of time on the mutex
    size_t mem = database_->memory();
    LOG(INFO) << "[SCENIC] [DATABASE] Memory Usage: " << mem/(1024.0*1024.0) << " MB";
}

Glider::Odometry ScenicOrchestrator::addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    glider_->addImu(timestamp, accel, gyro, quat);
    Glider::Odometry odom;
    if (current_state_.isInitialized()) {
        odom = glider_->interpolate(timestamp);
    }
    return odom;
}

Glider::OdometryWithCovariance ScenicOrchestrator::addGPS(int64_t timestamp, Eigen::Vector3d& gps)
{
    glider_->addGps(timestamp, gps);
    current_state_ = glider_->optimize(timestamp);
    return current_state_;
}

Glider::Odometry ScenicOrchestrator::getStateEstimate(int64_t timestamp)
{
    Glider::Odometry odom;
    if (current_state_.isInitialized()) {
        LOG_FIRST_N(INFO, 1) << "[SCENIC] [ORCHESTRATOR] State Estimation Initialized";
        odom = glider_->interpolate(timestamp);
    }
    return odom;
}

void ScenicOrchestrator::trackingCallback(int pid)
{
    LOG(INFO) << " [SCENIC] [TRACKING] Queue Size: " << tracking_processor_->size(Access::UNLOCK);
    if (pid > 0) {
        tracking_initialized_=true;
        LOG(INFO) << "[SCENIC] [TRACKING] Reprojection Error: " << database_->at<ScenicType::Reprojection>(pid);
        if (!stitching_processor_->contains(pid, Access::LOCK)) stitching_processor_->push(pid);
    } else {
        LOG(INFO) << "[SCENIC] [TRACKING] Waiting for Tracking to initialize";
    }
}

void ScenicOrchestrator::segmentationCallback(int pid)
{
    LOG(INFO) << "[SCENIC] [SEGMENTATION] Queue Size: " << seg_processor_->size(Access::UNLOCK);
    if (!stitching_processor_->contains(pid, Access::LOCK)) stitching_processor_->push(pid);
}

void ScenicOrchestrator::imageGraphCallback(std::shared_ptr<GraphWithPose> go)
{
    LOG(INFO) << "[SCENIC] [GRAPHING] Queue Size: " << graph_processor_->size(Access::UNLOCK);
    image_graph_ = go->graph;
    //stitching_processor_->push(*go);
}

void ScenicOrchestrator::graphCallback(std::shared_ptr<Graph> go)
{
    LOG(INFO) << "[SCENIC] [STITCHING] Queue Size: " << stitching_processor_->size(Access::UNLOCK);
    LOG(INFO) << "[SCENIC] [ORCHESTRATOR] Finished PID: " << go->getProcessID();
    // do something
    graph_ = go;
    new_graph_ = true;
}

bool ScenicOrchestrator::isInitialized() const
{
    return initialized_;
}

bool ScenicOrchestrator::isNewGraph() const
{
    return new_graph_;
}

cv::Mat ScenicOrchestrator::getGraphImage() const
{
    cv::Mat display;
    if (image_graph_) {
        cv::Mat img = getMaskOverlay();
        display = Graph::DrawGraph(image_graph_, img);
    }
    return display;
}

cv::Mat ScenicOrchestrator::getMaskOverlay() const
{
    int pid = graph_->getProcessID();
    cv::Mat img, mask;
    for (size_t& cid : database_->getCids()) {
        if (database_->at<ScenicType::Level>(cid) == GraphLevel::REGION) {
            img = database_->at<ScenicType::Image>(pid);
            mask = database_->at<ScenicType::Mask>(pid, cid);
        }
    }
    cv::Mat colored_mask;
    cv::cvtColor(mask, colored_mask, cv::COLOR_GRAY2BGR);
    colored_mask.setTo(cv::Scalar(255, 127, 41), mask);  // green

    cv::Mat result;
    cv::addWeighted(img, 1.0, colored_mask, 0.3, 0, result);

    return result;
}

void ScenicOrchestrator::setOrigin(double easting, double northing)
{
    origin_ = UTMPoint{easting, northing};
}

template <typename T>
T ScenicOrchestrator::exportToJsonFormat(const Graph& graph)
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

template Json::Value ScenicOrchestrator::exportToJsonFormat<Json::Value>(const Graph& graph);
template std::string ScenicOrchestrator::exportToJsonFormat<std::string>(const Graph& graph);

}// namespace ScenicOrchestrator
