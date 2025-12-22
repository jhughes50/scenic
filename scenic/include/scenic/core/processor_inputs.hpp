/*!
* @Author Jason Hughes
* @Date December 2025 
*
* @About keep all the input structs here
*/

#pragma once

#include <vector>
#include <string>
#include <cassert>
#include <functional>
#include <opencv2/opencv.hpp>
#include <glider/core/odometry.hpp>

namespace Scenic
{

struct Text
{
    size_t uid;
    std::string label;
    int level;
};

struct TextMap
{
    TextMap() = default;
    TextMap(const std::vector<std::pair<std::string, int>>& texts)
    {
        for (const std::pair<std::string, int> t : texts) {
            size_t uid = std::hash<std::string>{}(t.first);
            text.push_back(Text{uid, t.first, t.second});
        }
    }

    std::vector<std::string> getStrings() const
    {
        std::vector<std::string> label_str;
        for (const Text t : text) {
            label_str.push_back(t.label);
        }
        return label_str;
    }

    std::vector<Text> text;
};

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
    TextMap texts;

    SegmentationInput() = default;
    SegmentationInput(int id, const cv::Mat& img, const Glider::Odometry& odm, const TextMap& text)
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
    
    GraphingInput(const size_t size, cv::Mat img, TextMap text)
    {
        image = img;
        texts = text;
        heatmaps = std::vector<cv::Mat>(size);
        logits = std::vector<cv::Mat>(size);
        masks = std::vector<cv::Mat>(size);
    }
    
    GraphingInput(const size_t size, cv::Mat img, TextMap text, Glider::Odometry odm)
    {
        image = img;
        texts = text;
        odom = odm;
        heatmaps = std::vector<cv::Mat>(size);
        logits = std::vector<cv::Mat>(size);
        masks = std::vector<cv::Mat>(size);
    }

    size_t getSize() const
    {
        assert(logits.size() == texts.text.size() && "Logits and Texts are not the same size");
        assert(masks.size() == texts.text.size() && "Masks and Texts are not the same size");

        return texts.text.size();
    }

    std::vector<cv::Mat> heatmaps; // TODO depricated
    std::vector<cv::Mat> logits;
    std::vector<cv::Mat> masks;
};
} //namespace Scenic
