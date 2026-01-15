/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About orchestrate all the threads
*/

#pragma once

#include <glider/core/odometry.hpp>
#include <glider/core/glider.hpp>

#include "scenic/core/segmentation_processor.hpp"
#include "scenic/core/graphing_processor.hpp"
#include "scenic/core/tracking_processor.hpp"
#include "scenic/graphing/graph.hpp"
#include "scenic/utils/fixed_map.hpp"

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
        std::shared_ptr<Graph> getGraph();

        void push(const cv::Mat& img, const Glider::Odometry& odom); // dep

        void addImage(double vo_ts, int64_t gt_ts, const cv::Mat& img);
        void addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d gyro, Eigen::Vector4d quat);
        void addGPS(int64_t timestamp, Eigen::Vector3d& gps);

        void segmentationCallback(std::shared_ptr<GraphingInput> so);
        void trackingCallback(std::shared_ptr<TrackingOutput> to);
        void graphCallback(std::shared_ptr<Graph> go);
        
        bool isInitialized() const;
        bool isNewGraph() const;

        cv::Mat getGraphImage() const;

    private:
        TextMap texts_;
        std::shared_ptr<Graph> graph_;
    
        std::unique_ptr<SegmentationProcessor> seg_processor_;
        std::unique_ptr<GraphingProcessor> graph_processor_; 
        std::unique_ptr<TrackingProcessor> tracking_processor_;

        bool initialized_{false};
        bool new_graph_{false};

        FixedHashMap<int, cv::Mat> pid_img_map_;
        FixedHashMap<int, bool> pid_status_map_;
};
} // namespace Scenic
