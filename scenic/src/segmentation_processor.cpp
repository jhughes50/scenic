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
            Clipper::ClipperImageModelOutput image_output = model.setImage(processed_inputs.image);
            
            size_t input_size = inputs.getSize();
            std::array<cv::Mat, input_size> logits;
            std::array<cv::Mat, input_size> heatmaps;
            std::array<cv::Mat, input_size> masks;
            for (size_t i = 0; i < input_size; ++i) {
                at::Tensor raw_logits = model_.inference(image_output.activations,
                                                     inputs.tokens[i],
                                                     inputs.tokens[i]);
                
                cv::Mat cv_logits = processor_.postProcess(raw_logits);
                logits[i] = cv_logits;

                cv::Mat normalized_logits = processor_.normalize(cv_logits);
                heatmaps[i] = normalized_logits;
                
                // create mask
                cv::Mat mask;
                cv::threshold(normalized_logits, mask, 0.8, 1.0, cv::THRESH_BINARY);
                masks[i] = mask;
            }
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
