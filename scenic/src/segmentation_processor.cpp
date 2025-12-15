/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About process frames through the segmentation model
*/

#include "scenic/core/segmentation_processor.hpp"

using namespace Scenic;

SegmentationProcessor::SegmentationProcessor(size_t capacity, const std::string& model_path, const std::string& config_path) : ThreadedProcessor<SegmentationInput>(capacity)
{
    model_ = Clipper::ClipperModel(model_path);
    processor_ = Clipper::ClipperProcessor(config_path+"/clipper.yaml", config_path+"/merges.txt", config_path+"/vocab.json");
}

void SegmentationProcessor::setCallback(std::function<void(std::shared_ptr<SegmentationOutput>)> callback)
{
    outputCallback = callback;
}

void SegmentationProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<SegmentationInput> raw_input= pop(Access::PRELOCK);

            Clipper::ClipperModelInputs processed_inputs = processor_.process(raw_input->image, raw_input->texts);
            Clipper::ClipperModelOutput output = model_(processed_inputs);

            cv::Mat heatmap = processor_.postProcess(output.logits[0]);
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
