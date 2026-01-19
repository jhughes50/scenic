/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About a threaded wrapper around node graphing
*/
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "scenic/graphing/graph.hpp"
#include "scenic/graphing/traversability.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/processor_inputs.hpp"
#include "scenic/core/rectifier.hpp"

namespace Scenic
{

class StitchingProcessor : public ThreadedProcessor<Graph>
{
    public:
        StitchingProcessor() = default;
        StitchingProcessor(size_t capacity, const std::string& rect_path);

        void setCallback(std::function<void(std::shared_ptr<Graph>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<Graph>)> outputCallback;
        
        std::shared_ptr<Graph> scene_graph_;
        Rectifier rectifier_;
};
} // namespace Scenic
