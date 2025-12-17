/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About keep all the input structs here
*/

#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <glider/core/odometry.hpp>

namespace Scenic
{

struct BaseInput
{
    int pid;
    cv::Mat image;
    Glider::Odometry odom;

    virtual std::unique_ptr<BaseInput> clone() const 
    {
        return std::make_unique<BaseInput>(*this);
    }
};

struct TrackingInput : public BaseInput
{
    cv::Mat prev_image;
    Glider::Odometry prev_odom;
};

struct SegmentationInput : public BaseInput
{
    std::vector<std::string> texts;

    SegmentationInput() = default;
    SegmentationInput(int id, const cv::Mat& img, const Glider::Odometry& odm, std::vector<std::string>& text)
    {
        pid = id;
        image = img;
        odom = odm;
        texts = text;
    }
};

struct GraphingInput : public SegmentationInput
{
    GraphingInput() = default;
    GraphingInput(const size_t size)
    {
        heatmaps = std::vector<cv::Mat>(size);
        logits = std::vector<cv::Mat>(size);
        masks = std::vector<cv::Mat>(size);
    }
    
    GraphingInput(const size_t size, cv::Mat img, std::vector<std::string> text)
    {
        image = img;
        texts = text;
        heatmaps = std::vector<cv::Mat>(size);
        logits = std::vector<cv::Mat>(size);
        masks = std::vector<cv::Mat>(size);
    }
    
    GraphingInput(const size_t size, cv::Mat img, std::vector<std::string> text, Glider::Odometry odm)
    {
        image = img;
        texts = text;
        odom = odm;
        heatmaps = std::vector<cv::Mat>(size);
        logits = std::vector<cv::Mat>(size);
        masks = std::vector<cv::Mat>(size);
    }

    std::vector<cv::Mat> heatmaps;
    std::vector<cv::Mat> logits;
    std::vector<cv::Mat> masks;
};
} //namespace Scenic
