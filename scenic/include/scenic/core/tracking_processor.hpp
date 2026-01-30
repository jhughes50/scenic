/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About processor to run lgs
*/ 

#pragma once

#include <glog/logging.h>

#include "scenic/core/homography.hpp"
#include "scenic/core/rectifier.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/tracking_processor.hpp"
#include "scenic/core/database.hpp"

namespace Scenic 
{

struct TrackingInput
{
    int pid;
    cv::Mat previous;
    cv::Mat current;

    TrackingInput() = default;
};

class TrackingProcessor : public ThreadedProcessor<TrackingInput>
{
    public:
        TrackingProcessor() = default;
        TrackingProcessor(size_t capacity, const std::string& rpath, std::shared_ptr<ScenicDatabase> db);

        void setCallback(std::function<void(int)> callback);

    private:
        void processBuffer() override;
        std::function<void(int)> outputCallback;
    
        std::shared_ptr<ScenicDatabase> database_;
        Rectifier rectifier_;
};
}
