/*!
* @Author Jason Hughes
* @Date Decemeber 2025
*
* @About orchestrate all the threads
*/

#include <glog/logging.h>
#include "scenic/core/scenic.hpp"
#define LOG(severity) google::LogMessage(__FILE__, __LINE__, google::GLOG_##severity).stream()

namespace Scenic
{

Scenic::Scenic(size_t capacity, const std::string& model_path, const std::string& config_path)
{
    pid_img_map_.setCapacity(capacity);
    std::string params_path = "/usr/local/share/scenic/config/";
    seg_processor_ = std::make_unique<SegmentationProcessor>(capacity, params_path, model_path, config_path);
    seg_processor_->setCallback([this](std::shared_ptr<GraphingInput> so) { 
        this->segmentationCallback(so); 
    });

    graph_processor_ = std::make_unique<GraphingProcessor>(capacity, params_path+"blackfly-5mm.yaml");
    graph_processor_->setCallback([this](std::shared_ptr<Graph> go) {
        this->graphCallback(go);
    });

    tracking_processor_ = std::make_unique<TrackingProcessor>(capacity, params_path+"blackfly-5mm.yaml", params_path+"sticky-params.yaml");
    tracking_processor_->setCallback([this](std::shared_ptr<TrackingOutput> to) {
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
    tracking_processor_->start();
    seg_processor_->start();
    graph_processor_->start();
}

void Scenic::stop()
{
    LOG(INFO) << "[SCENIC] Stopping Scenic";
    tracking_processor_->stop();
    seg_processor_->stop();
    graph_processor_->stop();
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

void Scenic::push(const cv::Mat& img, const Glider::Odometry& odom)
{
    // create a pointer to input
    int pid = seg_processor_->generateProcessID();
    LOG(INFO) << "[SCENIC] Starting PID: " << pid;
    if (texts_.text.size() > 0) {
        pid_img_map_.insert(pid, img.clone());
        SegmentationInput seg_model_input(pid, img, odom, texts_);
        seg_processor_->push(seg_model_input);
    }
    else {
        LOG(FATAL) << "[SCENIC] Trying to inference before text was set";
    }
}

void Scenic::addImage(double vo_ts, int64_t gt_ts, const cv::Mat& img)
{
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    if (tracking_counter_ == 0) {
        prev_image_stamped_.image = gray;
        prev_image_stamped_.stampd = vo_ts;
        prev_image_stamped_.stampi = gt_ts;
    } else {
        int pid = tracking_processor_->generateProcessID();
        ImageStamped curr_image_stamped{gray, vo_ts, gt_ts};
        TrackingInput input{pid, prev_image_stamped_, curr_image_stamped};
        tracking_processor_->push(input);
        pid_status_map_[pid] = false;
        
        prev_image_stamped_ = curr_image_stamped;
        if (tracking_initialized_) {
            //TODO set segmentationInput and push to seg processor 
        }
    }
    tracking_counter_++;
}

void Scenic::graphCallback(std::shared_ptr<Graph> go)
{
    LOG(INFO) << "[SCENIC] Finished PID: " << go->getProcessID();
    graph_ = go;
    new_graph_ = true;
}

void Scenic::segmentationCallback(std::shared_ptr<GraphingInput> so)
{
    LOG(INFO) << "[SCENIC] Got Segmentation Output with PID " << so->pid;
    graph_processor_->push(*so);
}

void Scenic::trackingCallback(std::shared_ptr<TrackingOutput> to)
{
    if (to) {
        LOG_FIRST_N(INFO, 1) << "[SCENIC] Tacking Initialized";
        tracking_initialized_ = true;
    }
}

bool Scenic::isInitialized() const
{
    return initialized_;
}

bool Scenic::isNewGraph() const
{
    return new_graph_;
}

cv::Mat Scenic::getGraphImage() const
{
    cv::Mat display;
    if (graph_) {
        int pid = graph_->getProcessID();
        cv::Mat img = pid_img_map_.get(pid);
        display = Graph::DrawGraph(graph_, img);
    }
    return display;
}
}// namespace Scenic
