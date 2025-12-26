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

void Scenic::setText(const std::vector<std::pair<std::string,GraphLevel>>& text)
{

    texts_ = TextMap(text);
}

void Scenic::push(const cv::Mat& img, const Glider::Odometry& odom)
{
    // create a pointer to input
    int pid = seg_processor_->generateProcessID();
    if (texts_.text.size() > 0) {
        SegmentationInput seg_model_input(pid, img, odom, texts_);
        seg_processor_->push(seg_model_input);
    }
    else {
        // TODO shame user for trying to inference without setting text
    }
}

void Scenic::segmentationCallback(std::shared_ptr<GraphingInput> so)
{
    cv::imshow("mask", so->logits[0]);
    cv::waitKey(0);
    cv::Mat mask_8bit;
    so->masks[0].convertTo(mask_8bit, CV_8U, 255);  // Scale [0,1] to [0,255]
    cv::imwrite("test_mask.png", mask_8bit);
    // TODO push to grapher
}
}// namespace Scenic
