/*!
* Jason Hughes
* September 2025 
*
* Multi-threaded localizer
*/


#include "scenic/core/localization.hpp"

using namespace Scenic;

BufferLocalization::BufferLocalization(cv::Mat& c, cv::Mat& k, cv::Mat& d, Eigen::Isometry3d& p) : coords(c), intrinsics(k), distortion(d), pose(p) { }

BufferLocalization::BufferLocalization(BufferLocalization& bl, tbb::split) : coords(bl.coords), intrinsics(bl.intrinsics), distortion(bl.distortion), pose(bl.pose) { }

void BufferLocalization::operator()(const tbb::blocked_range2d<int>& r)
{
    for (int i = r.rows().begin(); i != r.rows().end(); ++i)
    {
        for (int j = r.cols().begin(); j != r.cols().end(); ++j)
        {
            cv::Point2d pixel(static_cast<double>(i), static_cast<double>(j));
            // reproject                    
            std::vector<cv::Point2d> undistorted;
            std::vector<cv::Point2d> pixel_vec = {pixel};
            cv::undistortPoints(pixel_vec, undistorted, intrinsics, distortion);
            Eigen::Vector3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0d);

            // ray cam to world
            Eigen::Vector3d ray_imu = transforms.T_imu_cam.linear() * ray_cam;
            Eigen::Vector3d ray_world = pose.linear() * ray_imu;

            // get sclae 
            double scale = -pose.translation().z() / ray_world(2);

            // scale and translate
            ray_imu = scale * ray_imu;
            Eigen::Vector3d pixel_imu = transforms.T_imu_cam.translation() + ray_imu;
            Eigen::Vector3d pixel_world = pose.translation() + (pose.linear() * pixel_imu);
            
            // add utm to 
            coords.at<cv::Vec2d>(i, j) = cv::Vec2d(pixel_world(0), pixel_world(1));
        }
    }
}

void BufferLocalization::join(const BufferLocalization& other) { }

void localizeNode(std::shared_ptr<Node>& node, const Eigen::Isometry3d& pose, cv::Mat K, cv::Mat D)
{
    Transforms transforms;
    cv::Point px = node->getPixelCoordinate();
    cv::Point2d pixel(static_cast<double>(px.x), static_cast<double>(px.y));
    std::vector<cv::Point2d> pixel_vec = {pixel};
    
    std::vector<cv::Point2d> undistorted;
    cv::undistortPoints(pixel_vec, undistorted, K, D);

    Eigen::Vector3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
    Eigen::Vector3d ray_imu = transforms.T_imu_cam.linear() * ray_cam;
    Eigen::Vector3d ray_world = pose.linear() * ray_imu;

    double scale = -pose.translation().z() / ray_world(2);

    ray_imu = scale * ray_imu;
    Eigen::Vector3d pixel_imu = transforms.T_imu_cam.translation() + ray_imu;
    Eigen::Vector3d pixel_world = pose.translation() + (pose.linear() * pixel_imu);
   
    node->setUtmCoordinate(pixel_world(0), pixel_world(1));
}

