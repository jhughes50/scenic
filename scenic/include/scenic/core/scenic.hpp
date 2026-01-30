/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About orchestrate all the threads
*/

#pragma once

#include <glider/core/odometry.hpp>
#include <glider/core/odometry_with_covariance.hpp>
#include <glider/core/glider.hpp>

#include "scenic/core/homography.hpp"
#include "scenic/core/segmentation_processor.hpp"
#include "scenic/core/graphing_processor.hpp"
#include "scenic/core/tracking_processor.hpp"
#include "scenic/core/stitching_processor.hpp"
#include "scenic/graphing/graph.hpp"
#include "scenic/utils/fixed_map.hpp"

namespace Scenic
{
class ScenicOrchestrator
{
    public:
        ScenicOrchestrator() = default;
        ScenicOrchestrator(size_t capacity, const std::string& model_path, const std::string& config_path);
        ~ScenicOrchestrator();

        void start();
        void stop();
        
        void setText(std::vector<Text> text);
        std::shared_ptr<Graph> getGraph();

        void push(int64_t timestamp, const cv::Mat& img, Glider::Odometry& odom); // dep

        Glider::Odometry addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat);
        Glider::OdometryWithCovariance addGPS(int64_t timestamp, Eigen::Vector3d& gps);
        
        Glider::Odometry getStateEstimate(int64_t timestamp);

        void segmentationCallback(int so);
        void trackingCallback(int to);
        void imageGraphCallback(std::shared_ptr<GraphWithPose> go);
        void graphCallback(std::shared_ptr<Graph> go);
        
        void setOrigin(double easting, double northing);

        bool isInitialized() const;
        bool isNewGraph() const;

        cv::Mat getGraphImage() const;
        cv::Mat getMaskOverlay() const;    

        template <typename T>
        T exportToJsonFormat(const Graph& graph);

    private:
        TextMap texts_;
        std::shared_ptr<Graph> graph_;
        std::shared_ptr<Graph> image_graph_;
        std::shared_ptr<ScenicDatabase> database_;
        
        std::unique_ptr<SegmentationProcessor> seg_processor_;
        std::unique_ptr<GraphingProcessor> graph_processor_; 
        std::unique_ptr<TrackingProcessor> tracking_processor_;
        std::unique_ptr<StitchingProcessor> stitching_processor_;

        bool initialized_{false};
        bool new_graph_{false};
        bool tracking_initialized_{false};
        cv::Mat prev_image_;
        std::map<int64_t, int> pid_stamped_;    

        UTMPoint origin_;

        std::shared_ptr<Glider::Glider> glider_;
        Glider::OdometryWithCovariance current_state_;
};
} // namespace Scenic
