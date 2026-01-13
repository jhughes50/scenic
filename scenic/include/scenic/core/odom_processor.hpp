/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About processor to run lgs
*/ 

#pragma once

#include <glog/logging.h>

#include "scenic/core/_processor.hpp"
#include "scenic/core/processor_inputs.hpp"

namespace Scenic 
{
class TrackingProcessor : public ThreadedProcessor<TrackingInput>
{
    public:
        TrackingProcessor() = default;
        TrackingProcessor(size_t capacity, const std::string& path);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<TrackingOutput>)> outputCallback);
};
}
