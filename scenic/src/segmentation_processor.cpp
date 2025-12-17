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

void SegmentationProcessor::setCallback(std::function<void(std::shared_ptr<GraphingInput>)> callback)
{
    outputCallback = callback;
}

void SegmentationProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<SegmentationInput> raw_input= pop(Access::PRELOCK);
            std::cout << "Using output: " << raw_input->texts[0] << std::endl;
            Clipper::ClipperModelInputs processed_inputs = processor_.process(raw_input->image, raw_input->texts);
            Clipper::ClipperImageModelOutput image_output = model_.setImage(processed_inputs.image);
            
            const size_t input_size = processed_inputs.getSize();
            //todo update with odom
            std::shared_ptr<GraphingInput> graphing_input = std::make_shared<GraphingInput>(input_size, raw_input->image, raw_input->texts);

            for (size_t i = 0; i < input_size; ++i) {
                at::Tensor raw_logits = model_.inference(image_output.activations,
                                                         processed_inputs.tokens[i],
                                                         processed_inputs.masks[i]);
               
                // convert the logits to cv and resize
                cv::Mat cv_logits = processor_.postProcess(raw_logits);
                double minVal, maxVal;
                cv::Point minLoc, maxLoc;
                cv::minMaxLoc(cv_logits, &minVal, &maxVal, &minLoc, &maxLoc);

                std::cout << "Min: " << minVal << " at " << minLoc << std::endl;
                std::cout << "Max: " << maxVal << " at " << maxLoc << std::endl;
                graphing_input->logits[i] = cv_logits.clone();

                // create mask
                cv::Mat mask;
                cv::threshold(cv_logits, mask, 1.0, 1.0, cv::THRESH_BINARY);
                graphing_input->masks[i] = mask.clone();
            }

            if (outputCallback) outputCallback(graphing_input);
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
