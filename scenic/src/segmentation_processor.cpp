/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About process frames through the segmentation model
*/

#include "scenic/core/segmentation_processor.hpp"

using namespace Scenic;

SegmentationProcessor::SegmentationProcessor(size_t capacity, std::string config_path, const std::string& model_path, const std::string& model_config_path) : ThreadedProcessor<SegmentationInput>(capacity)
{
    config_path += "/segmentation.yaml";
    params_ = SegmentationParameters::Load(config_path);
    model_ = Clipper::ClipperModel(model_path);
    processor_ = Clipper::ClipperProcessor(model_config_path+"/clipper.yaml", model_config_path+"/merges.txt", model_config_path+"/vocab.json");
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
            std::unique_ptr<SegmentationInput> raw_input = pop(Access::PRELOCK);
            std::vector<std::string> texts = raw_input->texts.getStrings(); 
            Clipper::ClipperModelInputs processed_inputs = processor_.process(raw_input->image, texts);
            Clipper::ClipperImageModelOutput image_output = model_.setImage(processed_inputs.image);
            const size_t input_size = processed_inputs.getSize();
            //todo update with odom
            std::vector<TextWithResults> text_map;
            for (size_t i = 0; i < input_size; i++) {
                std::cout << "Processing class: " << raw_input->texts.text[i].uid << std::endl;
                at::Tensor raw_logits = model_.inference(image_output.activations,
                                                         processed_inputs.tokens[i],
                                                         processed_inputs.masks[i]);
               
                // convert the logits to cv and resize
                cv::Mat cv_logits = processor_.postProcess(raw_logits);
                // create mask
                cv::Mat mask;
                switch (raw_input->texts.text[i].level) {
                    case OBJECT:
                        std::cout << "adding object" << std::endl;
                        cv::threshold(cv_logits, mask, params_.object_threshold, 1.0, cv::THRESH_BINARY);
                        break;
                    case REGION:
                        std::cout << "adding region" << std::endl;
                        cv::threshold(cv_logits, mask, params_.region_threshold, 1.0, cv::THRESH_BINARY);
                        break;
                }
                mask.convertTo(mask, CV_8U);
                TextWithResults results(raw_input->texts.text[i].uid,
                                        raw_input->texts.text[i].label,
                                        raw_input->texts.text[i].level,
                                        raw_input->texts.text[i].priority,
                                        cv_logits,
                                        mask);
                text_map.push_back(results);
            }
            std::shared_ptr<GraphingInput> graphing_input = std::make_shared<GraphingInput>(std::move(raw_input));
            graphing_input->map = text_map;
            if (outputCallback) outputCallback(graphing_input);
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
