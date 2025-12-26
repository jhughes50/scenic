/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About load parameters
*/

#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace Scenic
{
class Parameters
{
    public:
        Parameters() = default;
        Parameters(const std::string& path);

    protected:
        YAML::Node config_;
};

struct SegmentationParameters : public Parameters
{
    SegmentationParameters() = default;
    SegmentationParameters(const std::string& path);
    static SegmentationParameters Load(const std::string& path);

    float region_threshold{0.0};
    int region_id{0};
    float object_threshold{0.0};
    int object_id{0}; 
};  
}
