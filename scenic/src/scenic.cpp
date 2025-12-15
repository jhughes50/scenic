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
    seg_processor_ = std::make_unique<SegmentationProcessor>(capacity, model_path, config_path);
    seg_processor_->setCallback([this](std::shared_ptr<SegmentationOutput> so) { 
        this->segmentationCallback(so); 
    });
}

Scenic::~Scenic()
{
    stopScenic();
}

void Scenic::startScenic()
{
    seg_processor_->start();
}

void Scenic::stopScenic()
{
    seg_processor_->stop();
}   

void Scenic::push(const cv::Mat& img, const Glider::Odometry& odom)
{

}
} // namespace Scenic
