/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About a threaded wrapper around node graphing
*/
#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "scenic/graphing/graph.hpp"
#include "scenic/core/threaded_processor.hpp"
#include "scenic/core/processor_inputs.hpp"
#include "scenic/core/rectifier.hpp"
#include "scenic/core/stitching_inputs.hpp"
#include "scenic/utils/transforms.hpp"
#include "scenic/core/localization.hpp"
#include "scenic/core/buffer_search_coords.hpp"

namespace Scenic
{

class StitchingProcessor : public ThreadedProcessor<GraphWithPose>
{
    public:
        StitchingProcessor() = default;
        StitchingProcessor(size_t capacity, const std::string& rect_path);

        void setCallback(std::function<void(std::shared_ptr<Graph>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<Graph>)> outputCallback;
       
        // helper functions
        double calculateDistance(const UTMPoint& p1, const UTMPoint& p2);
        void localizeNodes(std::shared_ptr<Graph>& graph, const Eigen::Isometry3d& pose);   
        void checkRegionNodes(const std::shared_ptr<Graph>& graph);
        void checkObjectNodes(const std::shared_ptr<Graph>& graph);
        void regionRegistrationViaBackProjection(const cv::Mat& coords, const std::shared_ptr<Graph>& graph, const GraphingInput& gi);


        std::shared_ptr<Graph> scene_graph_;
        Rectifier rectifier_;
        Transforms transforms_;
};
} // namespace Scenic
