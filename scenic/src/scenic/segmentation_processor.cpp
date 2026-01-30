/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About process frames through the segmentation model
*/

#include "scenic/core/segmentation_processor.hpp"

using namespace Scenic;

SegmentationProcessor::SegmentationProcessor(size_t capacity, std::string config_path, const std::string& model_path, const std::string& model_config_path, std::shared_ptr<ScenicDatabase> db) : ThreadedProcessor<SegmentationInput>(capacity)
{
    database_ = db;
    config_path += "/segmentation.yaml";
    params_ = SegmentationParameters::Load(config_path);
    model_ = Clipper::ClipperModel(model_path);
    processor_ = Clipper::ClipperProcessor(model_config_path+"/clipper.yaml", model_config_path+"/merges.txt", model_config_path+"/vocab.json");
}

void SegmentationProcessor::setCallback(std::function<void(int)> callback)
{
    outputCallback = callback;
}

void SegmentationProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<SegmentationInput> raw_input = pop(Access::PRELOCK);
            int pid = raw_input->pid;

            std::vector<std::string> texts;
            std::map<int, size_t> iter_cid_map;
            int it = 0;
            for (size_t cid : database_->getCids()) {
                texts.push_back(database_->at<ScenicType::Class>(cid));
                iter_cid_map[it] = cid;
                it++;
            }

            Clipper::ClipperModelInputs processed_inputs = processor_.process(raw_input->image, texts);
            Clipper::ClipperImageModelOutput image_output = model_.setImage(processed_inputs.image);
            
            const size_t input_size = processed_inputs.getSize();
            //assert(input_size == database_->getCids().size(), "Big Problem");

            for (size_t i = 0; i < input_size; i++) {
                size_t cid = iter_cid_map[i];
                at::Tensor raw_logits = model_.inference(image_output.activations,
                                                         processed_inputs.tokens[i],
                                                         processed_inputs.masks[i]);
               
                // convert the logits to cv and resize
                cv::Mat cv_logits = processor_.postProcess(raw_logits);
                // create mask
                cv::Mat mask;
                switch (database_->at<ScenicType::Level>(cid)) {
                    case OBJECT:
                        cv::threshold(cv_logits, mask, params_.object_threshold, 1.0, cv::THRESH_BINARY);
                        break;
                    case REGION:
                        cv::threshold(cv_logits, mask, params_.region_threshold, 1.0, cv::THRESH_BINARY);
                        break;
                }
                mask.convertTo(mask, CV_8U);

                database_->insert<ScenicType::Mask>(pid, cid, mask);
                database_->insert<ScenicType::Logits>(pid, cid, cv_logits);
            }
            if (outputCallback) outputCallback(pid);
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
