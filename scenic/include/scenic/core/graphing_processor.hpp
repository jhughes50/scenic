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
#include "scenic/core/stitching_inputs.hpp"

namespace Scenic
{

class GraphingProcessor : public ThreadedProcessor<GraphingInput>
{
    public:
        GraphingProcessor() = default;
        GraphingProcessor(size_t capacity, const std::string& rect_path);

        void setCallback(std::function<void(std::shared_ptr<GraphWithPose>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<GraphWithPose>)> outputCallback;

        KMeans kmeans_;
};
} // namespace Scenic
