/*!
* @Author Jason Hughes
* @Date Septemeber 2025
*
*/

#pragma once

#include <Eigen/Dense>

namespace Scenic
{
struct Transforms
{
    Eigen::Matrix3d T_ned_enu;
    Eigen::Isometry3d T_cam_imu;
    Eigen::Isometry3d T_imu_cam;
    Eigen::Isometry3d T_map_utm;

    Transforms()
    {
        T_map_utm = Eigen::Isometry3d::Identity();
        T_ned_enu = Eigen::Matrix3d::Zero();
        T_ned_enu(1, 0) = -1.0;
        T_ned_enu(0, 1) = 1.0;
        T_ned_enu(2, 2) = -1.0;

        T_cam_imu.translation() << 0.036, -0.008, -0.0877;
        T_cam_imu.linear() = (Eigen::Matrix3d() << 0.0, -1.0, 0.0,
                                                   -1.0, 0.0, 0.0, 
                                                   0.0, 0.0, -1.0).finished();
        T_imu_cam = T_cam_imu.inverse();
    }

    void setUTMToMapTransform(std::pair<double, double> utm)
    {
        T_map_utm.translation() << utm.first, utm.second, 0;
    }
};
}
