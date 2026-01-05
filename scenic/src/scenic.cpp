/*!
* @Author Jason Hughes
* @Date Decemeber 2025
*
* @About orchestrate all the threads
*/

#include "scenic/core/scenic.hpp"

namespace Scenic
{

Scenic::Scenic(size_t capacity, const std::string& model_path, const std::string& config_path)
{
    std::string params_path = "../config/";
    seg_processor_ = std::make_unique<SegmentationProcessor>(capacity, params_path, model_path, config_path);
    seg_processor_->setCallback([this](std::shared_ptr<GraphingInput> so) { 
        this->segmentationCallback(so); 
    });

    graph_processor_ = std::make_unique<GraphingProcessor>(capacity);
    graph_processor_->setCallback([this](std::shared_ptr<Graph> go) {
        this->graphCallback(go);
    });
}

Scenic::~Scenic()
{
    stop();
}

void Scenic::start()
{
    seg_processor_->start();
}

void Scenic::stop()
{
    seg_processor_->stop();
}   

void Scenic::setText(std::vector<Text> text)
{
    for (const Text& t : text) {
        std::cout << " Class: " << t.label << " with uid: " << t.uid << std::endl;
    }

    texts_.text = text;
}

Graph Scenic::getGraph() const
{
    return graph_;
}

void Scenic::push(const cv::Mat& img, const Glider::Odometry& odom)
{
    // create a pointer to input
    std::cout << "Got Image Odom Pair" << std::endl;
    int pid = seg_processor_->generateProcessID();
    if (texts_.text.size() > 0) {
        SegmentationInput seg_model_input(pid, img, odom, texts_);
        seg_processor_->push(seg_model_input);
    }
    else {
        std::cerr << "[SCENIC] Trying to inference before text was set" << std::endl;
    }
}

void Scenic::graphCallback(std::shared_ptr<Graph> go)
{
    std::cout << "Got a Graph" << std::endl;
    graph_ = *go; 
}

void Scenic::segmentationCallback(std::shared_ptr<GraphingInput> so)
{
    std::cout << "Got Segmentation Output" << std::endl;
    for (const TextWithResults& t : so->map) {
        std::cout << "Have class: " << t.label << " with uid " << t.uid << std::endl;
    }
    graph_processor_->push(*so);
}
}// namespace Scenic
