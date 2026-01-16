/*
* Jason Hughes
* June 2025
*
*/
#include <cassert>
#include "scenic/core/rectifier.hpp"

using namespace Scenic;

Rectifier::Rectifier(std::string path, int scale)
{
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        auto cam = config["cam0"];
        auto intrinsics = cam["intrinsics"];
        intrinsics_ = (cv::Mat_<double>(3,3) << intrinsics[0].as<double>(), 0, intrinsics[2].as<double>(),
                                                0, intrinsics[1].as<double>(), intrinsics[3].as<double>(),
                                                0, 0, 1);

        auto distortion = cam["distortion_coeffs"];
        distortion_ = cv::Mat::zeros(4,1,CV_64F);
        for (size_t ind=0; ind<4; ind++)
        {
            distortion_.at<double>(ind, 0) = cam["distortion_coeffs"][ind].as<double>();
        }
        

        width_ = cam["resolution"][0].as<int>();
        height_ = cam["resolution"][1].as<int>();
        resolution_ = cv::Size(width_, height_);
        output_res_ = cv::Size(width_*0.5, height_*0.5);
        calculateOutputIntrinsics();
        
        auto t_body_cam = cam["t_body_cam"];
        Eigen::Vector3d t(t_body_cam[0].as<double>(), t_body_cam[1].as<double>(), t_body_cam[2].as<double>());
        setTransformBodyCam(t);

        hfov_ = cam["hfov"].as<double>();
        vfov_ = cam["vfov"].as<double>();
        
    }
    catch (const std::exception& e)
    {
        LOG(FATAL) << "[SCENIC] Error loading calibration file at " << path << ", with error: " << e.what();
    }
}

Rectifier Rectifier::Load(std::string path, int scale)
{
    Rectifier rect;
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        auto cam = config["cam0"];
        auto intrinsics = cam["intrinsics"];
        cv::Mat intrin = (cv::Mat_<double>(3,3) << intrinsics[0].as<double>()/scale, 0, intrinsics[2].as<double>()/scale,
                                                0, intrinsics[1].as<double>()/scale, intrinsics[3].as<double>()/scale,
                                                0, 0, 1);
        rect.setIntrinsics(intrin);

        auto distortion = cam["distortion_coeffs"];
        cv::Mat dist = cv::Mat::zeros(4,1,CV_64F);
        for (size_t ind=0; ind<4; ind++)
        {
            dist.at<double>(ind, 0) = cam["distortion_coeffs"][ind].as<double>();
        }
        
        rect.setDistortion(dist);

        int width = cam["resolution"][0].as<int>()/scale;
        int height = cam["resolution"][1].as<int>()/scale;
        //auto t_body_cam = cam["T_body_cam"];
        //Eigen::Vector3d t(t_body_cam[0].as<double>(), t_body_cam[1].as<double>(), t_body_cam[2].as<double>());

        //rect.setTransformBodyCam(t);
        rect.setResolution(width, height);
        rect.calculateOutputIntrinsics();

        double hfov = cam["hfov"].as<double>();
        double vfov = cam["vfov"].as<double>();
        rect.setHorizontalFov(hfov);
        rect.setVerticalFov(vfov);
        LOG(INFO) << "[SCENIC] Loaded Camera Params from: " << path;
    }
    catch (const std::exception& e)
    {
        LOG(FATAL) << "[SCENIC] Error loading calibration file at " << path << ", with error: " << e.what();
    }

    return rect;
}

std::vector<Eigen::Vector2d> Rectifier::undistortPixelPoints(const std::vector<Eigen::Vector2d>& px)
{
    std::vector<Eigen::Vector2d> out_px;
    out_px.reserve(px.size());

    std::vector<cv::Point2f> src;
    src.reserve(src.size());
    for (const Eigen::Vector2d& p : px) src.emplace_back((float)p.x(), (float)p.y());

    std::vector<cv::Point2f> dst;
    cv::undistortPoints(src, dst, intrinsics_, distortion_, cv::noArray(), intrinsics_);
    
    for (const cv::Point2f& p : dst) out_px.emplace_back((double)p.x, (double)p.y);

    return out_px;
}

void Rectifier::rectifyMonoImage(const cv::Mat& input, cv::Mat& output) const
{
    static cv::Mat map1, map2;
    static bool maps_initialized = false;
    
    if (!maps_initialized)
    {
        cv::initUndistortRectifyMap(intrinsics_, distortion_, cv::Mat(),
                                    intrinsics_, cv::Size(width_, height_),
                                    CV_16SC2, map1, map2);
        maps_initialized = true;
    }
    
    cv::remap(input, output, map1, map2, cv::INTER_LINEAR);
}

void Rectifier::setIntrinsics(const cv::Mat& i)
{
    intrinsics_ = i;
}

void Rectifier::setDistortion(const cv::Mat& d)
{
    distortion_ = d;
}

void Rectifier::setResolution(const int& w, const int& h)
{
    width_ = w;
    height_ = h;
    resolution_ = cv::Size(w, h);
    output_res_ = cv::Size(w/2, h/2);
}

void Rectifier::setWidth(const int& w)
{
    width_ = w;
}

void Rectifier::setHeight(const int& h)
{
    height_ = h;
}

void Rectifier::setHorizontalFov(const double& hfov)
{
    hfov_ = hfov;
}

void Rectifier::setVerticalFov(const double& vfov)
{
    vfov_ = vfov;
}

void Rectifier::calculateOutputIntrinsics()
{
    // keep the input size and the output size the same as the resolution for now.
    output_intrinsics_ = cv::getOptimalNewCameraMatrix(intrinsics_, distortion_, resolution_, 0, output_res_);
}

void Rectifier::setTransformBodyCam(const Eigen::Vector3d& t)
{
    T_body_cam_ = Eigen::Isometry3d::Identity();
    T_body_cam_.translation() = t;
}

Eigen::Isometry3d Rectifier::getTransformBodyCam() const
{
    return T_body_cam_;
}

template<typename T>
T Rectifier::getHeight() const
{
    if constexpr (std::is_same_v<T, int>)
    {
        return static_cast<int>(height_);
    }
    else
    {
        return height_;
    }
}

template<typename T>
T Rectifier::getWidth() const
{
    if constexpr (std::is_same_v<T, int>)
    {
        return static_cast<int>(width_);
    }
    else
    {
        return width_;
    }
}  

template<typename T>
T Rectifier::getIntrinsics() const
{
    if constexpr (std::is_same_v<T, Eigen::Matrix3d>)
    {
        Eigen::Matrix3d k;
        for (int i = 0; i < 3; ++i) 
        {
            for (int j = 0; j < 3; ++j) 
            {
                k(i, j) = intrinsics_.at<double>(i, j);
            }
        }
        return k;
    }
    else
    {
        return intrinsics_;
    }
}

template<typename T>
T Rectifier::getOutputIntrinsics() const
{
    if constexpr (std::is_same_v<T, Eigen::Matrix3d>)
    {
        Eigen::Matrix3d k;
        for (int i = 0; i < 3; ++i) 
        {
            for (int j = 0; j < 3; ++j) 
            {
                k(i, j) = output_intrinsics_.at<double>(i, j);
            }
        }
        return k;
    }
    else
    {
        return output_intrinsics_;
    }
}

cv::Size Rectifier::getResolution() const
{
    cv::Size res(width_, height_);

    return res;
}

cv::Size Rectifier::getOutputResolution() const
{
    return output_res_;
}

double Rectifier::getHorizontalFov() const
{
    return hfov_;
}

double Rectifier::getVerticalFov() const
{
    return vfov_;
}


template int Rectifier::getHeight<int>() const;
template size_t Rectifier::getHeight<size_t>() const;

template int Rectifier::getWidth<int>() const;
template size_t Rectifier::getWidth<size_t>() const;

template cv::Mat Rectifier::getIntrinsics<cv::Mat>() const;
template Eigen::Matrix3d Rectifier::getIntrinsics<Eigen::Matrix3d>() const;

template cv::Mat Rectifier::getOutputIntrinsics<cv::Mat>() const;
template Eigen::Matrix3d Rectifier::getOutputIntrinsics<Eigen::Matrix3d>() const;
