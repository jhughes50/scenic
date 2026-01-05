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
enum GraphLevel
{
    OBJECT,
    REGION
};

enum RegionPriority
{
    HIGH,
    MEDIUM,
    LOW,
    NONE
};

struct Text
{
    Text() = default;
    Text(size_t u, std::string lb, GraphLevel lv, RegionPriority p)
    {       
        uid = u;
        label = lb;
        level = lv;
        priority = p;
        switch(p) {
            case HIGH:
                multiplier = 1.0;
                break;
            case MEDIUM:
                multiplier = 0.66;
                break;
            case LOW:
                multiplier = 0.33;
                break;
            case NONE:
                multiplier = 0.0;
                break;
            default:
                multiplier = 0.0;
        }
    }

    Text(std::string lb, GraphLevel lv, RegionPriority p)
    {
        uid =  std::hash<std::string>{}(lb);
        label = lb;
        level = lv;
        priority = p;
        switch(p) {
            case HIGH:
                multiplier = 1.0;
                break;
            case MEDIUM:
                multiplier = 0.66;
                break;
            case LOW:
                multiplier = 0.33;
                break;
            case NONE:
                multiplier = 0.0;
                break;
            default:
                multiplier = 0.0;
        }
    }
    
    Text(size_t u, std::string lb, GraphLevel lv)
    {       
        uid = u;
        label = lb;
        level = lv;
        assert(lv != GraphLevel::REGION && "Regions need to set a RegionPriority");
        priority = RegionPriority::NONE;
        multiplier = 0.0;
    }

    size_t uid;
    std::string label;
    GraphLevel level;
    RegionPriority priority;
    float multiplier;
};

struct TextWithResults : public Text
{
    TextWithResults() = default;
    TextWithResults(size_t uid, std::string label, GraphLevel level, RegionPriority p, cv::Mat log, cv::Mat mas) : Text(uid, label, level, p)
    {
        logits = log;
        mask = mas;
    }

    cv::Mat logits;
    cv::Mat mask;
};

struct TextMap
{
    TextMap() = default;
    TextMap(const std::vector<std::pair<std::string, GraphLevel>>& texts)
    {
        for (const std::pair<std::string, GraphLevel> t : texts) {
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

struct GraphingInput : public BaseInput
{
    GraphingInput() = default;

    GraphingInput(std::unique_ptr<SegmentationInput> input)
    { 
        pid = input->pid;
        odom = input->odom;
        image = input->image;
    }

    size_t getSize() const
    {
        return map.size();
    }

    cv::Mat getLogitsFromClassLabel(int label) const
    {
        cv::Mat logits;
        for (const TextWithResults& e : map) {
            if (e.uid == label) logits = e.logits;
        }
        return logits;
    }

    cv::Mat getNormalizedLogitsFromClassLabel(int label) const
    {
        std::cout << "Getting Logits for class label: " << label << std::endl;
        cv::Mat logits;
        for (const TextWithResults& e : map) {
            if (e.uid == label) logits = e.logits;
        }
        cv::normalize(logits, logits, 0.0, 1.0, cv::NORM_MINMAX);

        return logits;
    }

    cv::Mat getMaskFromClassLabel(int label) const
    {
        cv::Mat mask;
        for (const TextWithResults& e : map) {
            if (e.uid == label) mask = e.mask;
        }
        return mask;
    }

    float getMultiplierFromClassLabel(int label) const
    {
        float mult = 0.0;
        for (const TextWithResults& e : map) {
            if (e.uid == label) {
                mult = e.multiplier;
            }
        }
        return mult; 
    }

    cv::Mat image;
    std::vector<TextWithResults> map; 
};
} //namespace Scenic
