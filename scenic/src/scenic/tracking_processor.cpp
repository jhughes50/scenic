/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About process frames through stickyVO
*/

#include "scenic/core/homography.hpp"
#include "scenic/core/tracking_processor.hpp"

using namespace Scenic;

TrackingProcessor::TrackingProcessor(size_t capacity, const std::string& rect_path, const std::string& params_path) : ThreadedProcessor<TrackingInput>(capacity)
{
    rectifier_ = Rectifier::Load(rect_path, 4);

    cv::Mat k = rectifier_.getIntrinsics<cv::Mat>();
}

void TrackingProcessor::setCallback(std::function<void(std::shared_ptr<homography::HomographyResult>)> callback)
{
    outputCallback = callback;
}

void TrackingProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<TrackingInput> raw_input = pop(Access::PRELOCK);

            auto output = homography::compute_homography(raw_input->previous.image, raw_input->current.image);
            
            if (output) {
                auto result = std::make_shared<homography::HomographyResult>(*output);
                outputCallback(result);
            }
        } else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
