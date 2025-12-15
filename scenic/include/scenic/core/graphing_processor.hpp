/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About a threaded wrapper around node graphing
*/

#include <opencv2/opencv.hpp>

#include "scenic/core/threaded_processor.hpp"

namespace Scenic
{
struct GraphInput
{
    cv::Mat image;
};

class GraphingProcessor : ThreadedProcessor<GraphInput>
{
    public:
        GraphingProcessor() = default;
        GraphingProcessor(size_t capacity);

        void setCallback(std::function<void(std::shared_ptr<GraphInput>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<GraphInput>)> outputCallback;
};
} // namespace Scenic
