/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About processor to run lgs
*/ 

#pragma once

#include <glog/logging.h>

#include "scenic/core/rectifier.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/tracking_processor.hpp"
#include "scenic/core/processor_inputs.hpp"
#include "scenic/core/tracking_inputs.hpp"

#include "stickyvo_lgs/lgs_frontend.hpp"
#include "stickyvo/core.hpp"

namespace Scenic 
{
class TrackingProcessor : public ThreadedProcessor<TrackingInput>
{
    public:
        TrackingProcessor() = default;
        TrackingProcessor(size_t capacity, const std::string& rpath, const std::string& ppath);

        void setCallback(std::function<void(std::shared_ptr<TrackingOutput>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<TrackingOutput>)> outputCallback;

        Rectifier rectifier_;
        std::unique_ptr<stickyvo_lgs::LgsFrontend> lgs_;
        std::unique_ptr<stickyvo::StickyVoCore> sticky_core_;

        stickyvo_lgs::CameraIntrinsics K_;
        stickyvo::CameraIntrinsics K_vo_;

        int bootstrap_fail_count_{0};
        int tracking_fail_count_{0};
        double qw_{1.0}, qx_{0.0}, qy_{0.0}, qz_{0.0};
};
}
