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

namespace Scenic
{

struct SegmentationInput 
{
    cv::Mat image;
    std::vector<std::string> texts;
};

struct SegmentationOutput : public SegmentationInput
{
    std::vector<cv::Mat> logits;
    std::vector<cv::Mat> heatmaps;
};

class SegmentationProcessor : public ThreadedProcessor<SegmentationInput>
{
    public:
        SegmentationProcessor() = default;
        SegmentationProcessor(size_t capacity, const std::string& model_path, const std::string& config_path);

        void setTextInputs(std::vector<std::string> t);
        void setCallback(std::function<void(std::shared_ptr<SegmentationOutput>)> callback);

    private:
        void processBuffer() override;

        Clipper::ClipperModel model_;
        Clipper::ClipperProcessor processor_;

        std::vector<std::string> texts_;

        std::function<void(std::shared_ptr<SegmentationOutput>)> outputCallback;
};
} //namespace Scenic
