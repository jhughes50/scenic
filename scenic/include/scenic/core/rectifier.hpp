/*
* Jason Hughes
* June 2025
*
* rectifier object
*/

#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace Scenic
{

class Rectifier
{
    public:
        Rectifier() = default;
        Rectifier(std::string path, int scale = 1);

        static Rectifier Load(std::string path, int scale = 1);

        void setIntrinsics(const cv::Mat& i);
        void setDistortion(const cv::Mat& d);
        void setTranslation(const cv::Mat& t);
        void setTransformBodyCam(const Eigen::Vector3d& t);

        void setResolution(const int& w, const int& h);
        void setWidth(const int& w);
        void setHeight(const int& h);

        void setHorizontalFov(const double& hfov);
        void setVerticalFov(const double& vfov);

        void rectifyMonoImage(const cv::Mat& input, cv::Mat& output) const;
        void calculateOutputIntrinsics();

        std::vector<Eigen::Vector2d> undistortPixelPoints(const std::vector<Eigen::Vector2d>& in_px);

        template<typename T>
        T getWidth() const;
        template<typename T>
        T getHeight() const;

        template<typename T>
        T getIntrinsics() const;
        template<typename T>
        T getDistortion() const;
        template<typename T>
        T getOutputIntrinsics() const;
        double getFocalLength() const;

        cv::Size getResolution() const;
        cv::Size getOutputResolution() const;
        Eigen::Isometry3d getTransformBodyCam() const;

        double getHorizontalFov() const;
        double getVerticalFov() const;

    private:
        cv::Mat intrinsics_;
        cv::Mat distortion_;
        cv::Mat output_intrinsics_;

        Eigen::Isometry3d T_body_cam_;

        size_t height_;
        size_t width_;
        cv::Size resolution_;
        cv::Size output_res_;
        
        double hfov_;
        double vfov_;
};
}
