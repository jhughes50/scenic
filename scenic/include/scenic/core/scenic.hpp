/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About orchestrate all the threads
*/

#pragma once

#include <glider/core/odometry.hpp>


#include "scenic/core/segmentation_processor.hpp"
#include "scenic/core/graphing_processor.hpp"
#include "scenic/graphing/graph.hpp"

namespace Scenic
{
class Scenic
{
    public:
        Scenic() = default;
        Scenic(size_t capacity, const std::string& model_path, const std::string& config_path);
        ~Scenic();

        void start();
        void stop();
        
        void setText(std::vector<Text> text);
        Graph getGraph() const;
        void push(const cv::Mat& img, const Glider::Odometry& odom);

        void segmentationCallback(std::shared_ptr<GraphingInput> so);
        void graphCallback(std::shared_ptr<Graph> go);        
        // todo implement these
        //void homographyCallback(nullptr);

    private:
        TextMap texts_;
        Graph graph_;
        std::unique_ptr<SegmentationProcessor> seg_processor_;
        std::unique_ptr<GraphingProcessor> graph_processor_;  
};
} // namespace Scenic
