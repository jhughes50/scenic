/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About io for tracking processor
*/

#include <Eigen/Dense>
#include <glider/core/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <stickyvo/types.hpp>

namespace Scenic
{
struct ImageStamped
{
    cv::Mat image;
    Glider::Odometry odom;
    double stampd;
    int64_t stampi;


struct TrackingOutput
{
    TrackingOutput() = default;
    TrackingOutput(int id, int64_t si, double sd, Eigen::Vector3d p, Eigen::Quaterniond q, std::vector<stickyvo::FeatureObs> f)
    {
        pid = id;
        stampi = si;
        stampd = sd; 
        position = p;
        orientation = q;
        features = f;
    }

    int pid;
    int64_t stampi;
    double stampd;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    std::vector<stickyvo::FeatureObs> features;
};
};
