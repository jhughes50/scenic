/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About a threaded wrapper around node graphing
*/
#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <glider/core/glider.hpp>

#include "scenic/graphing/graph.hpp"
#include "scenic/core/rectifier.hpp"
#include "scenic/utils/transforms.hpp"
#include "scenic/core/localization.hpp"
#include "scenic/core/buffer_search_coords.hpp"
#include "scenic/graphing/kmeans.hpp"
#include "scenic/graphing/traversability.hpp"
#include "scenic/core/stitching_inputs.hpp"
#include "scenic/core/database.hpp"
#include "scenic/core/threaded_processor.hpp"

namespace Scenic
{

class StitchingProcessor : public ThreadedProcessor<int>
{
    public:
        StitchingProcessor() = default;
        StitchingProcessor(size_t capacity, const std::string& rect_path, std::shared_ptr<Glider::Glider>& glider, std::shared_ptr<ScenicDatabase> db);

        void setCallback(std::function<void(std::shared_ptr<Graph>)> callback);

    private:
        void processBuffer() override;
        std::function<void(std::shared_ptr<Graph>)> outputCallback;
       
        // helper functions
        double calculateDistance(const UTMPoint& p1, const UTMPoint& p2);
        void checkRegionNodes(const std::shared_ptr<Graph>& graph);
        void checkObjectNodes(const std::shared_ptr<Graph>& graph, const Glider::Odometry& pose);
        void addNodeByMax(const GraphWithPose& input);

        
        std::shared_ptr<Glider::Glider> glider_;
        std::shared_ptr<Graph> scene_graph_;
        std::shared_ptr<ScenicDatabase> database_;    

        Rectifier rectifier_;
        Transforms transforms_;
        KMeans kmeans_;
};
} // namespace Scenic
