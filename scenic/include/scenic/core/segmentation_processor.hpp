/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About Wrap the segmentation model in a 
* threaded processor.
*/

#include <opencv2/opencv.hpp>
#include <clipper/clipper_model.hpp>

#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/processor_inputs.hpp"

namespace Scenic
{
class SegmentationProcessor : public ThreadedProcessor<SegmentationInput>
{
    public:
        SegmentationProcessor() = default;
        SegmentationProcessor(size_t capacity, const std::string& model_path, const std::string& config_path);

        void setTextInputs(std::vector<std::string> t);
        void setCallback(std::function<void(std::shared_ptr<GraphingInput>)> callback);

    private:
        void processBuffer() override;

        Clipper::ClipperModel model_;
        Clipper::ClipperProcessor processor_;

        // we output the input to the graphing thread
        std::function<void(std::shared_ptr<GraphingInput>)> outputCallback;
};
} //namespace Scenic
