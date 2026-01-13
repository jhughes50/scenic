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


namespace Scenic
{

class GraphingProcessor : public ThreadedProcessor<GraphingInput>
{
    public:
        GraphingProcessor() = default;
        GraphingProcessor(size_t capacity, const std::string& rect_path);

        void setCallback(std::function<void(std::shared_ptr<Graph>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<Graph>)> outputCallback;

        KMeans kmeans_;
};
} // namespace Scenic
