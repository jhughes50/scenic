/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About a threaded wrapper around node graphing
*/

#include <opencv2/opencv.hpp>

#include "scenic/graphing/graph.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/processor_inputs.hpp"

namespace Scenic
{

class GraphingProcessor : ThreadedProcessor<GraphingInput>
{
    public:
        GraphingProcessor() = default;
        GraphingProcessor(size_t capacity);

        void setCallback(std::function<void(std::shared_ptr<ScenicGraph>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<ScenicGraph>)> outputCallback;
};
} // namespace Scenic
