/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About Wrap the segmentation model in a 
* threaded processor.
*/
#include <cassert>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <clipper/clipper_model.hpp>

#include "scenic/core/database.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/utils/params.hpp"

namespace Scenic
{

struct SegmentationInput
{
    int pid;
    cv::Mat image;
};

class SegmentationProcessor : public ThreadedProcessor<SegmentationInput>
{
    public:
        SegmentationProcessor() = default;
        SegmentationProcessor(size_t capacity, 
                              std::string config_path, 
                              const std::string& model_path,
                              const std::string& model_config_path, 
                              std::shared_ptr<ScenicDatabase> db);

        void setTextInputs(std::vector<std::string> t);
        void setCallback(std::function<void(int)> callback);

    private:
        void processBuffer() override;

        std::shared_ptr<ScenicDatabase> database_;
        Clipper::ClipperModel model_;
        Clipper::ClipperProcessor processor_;
        SegmentationParameters params_;
        // we output the input to the graphing thread
        std::function<void(int)> outputCallback;
};
} //namespace Scenic
