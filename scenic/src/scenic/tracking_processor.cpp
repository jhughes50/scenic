/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About process frames through stickyVO
*/

#include "scenic/core/homography.hpp"
#include "scenic/core/tracking_processor.hpp"

using namespace Scenic;

TrackingProcessor::TrackingProcessor(size_t capacity, const std::string& rect_path, std::shared_ptr<ScenicDatabase> db) : ThreadedProcessor<TrackingInput>(capacity)
{
    rectifier_ = Rectifier::Load(rect_path, 4);
    database_ = db;

    cv::Mat k = rectifier_.getIntrinsics<cv::Mat>();
}

void TrackingProcessor::setCallback(std::function<void(int)> callback)
{
    outputCallback = callback;
}

void TrackingProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<TrackingInput> raw_input = pop(Access::PRELOCK);

            auto output = computeHomography(raw_input->previous, raw_input->current);
            
            if (output) {
                HomographyResult result = *output;
                
                int pid = raw_input->pid;
                database_->insert<ScenicType::Homography>(pid, result.H);
                database_->insert<ScenicType::Reprojection>(pid, result.reprojection_error);
                database_->insert<ScenicType::Features>(pid, result.inliers);
                
                outputCallback(pid);
            } else {
                outputCallback(0);
            }
        } else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
