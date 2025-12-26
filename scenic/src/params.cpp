/*!
* @Author Jason Hughe
* @Date December 2025
*
* @About load parameters
*/

#include "scenic/utils/params.hpp"

using namespace Scenic;

Parameters::Parameters(const std::string& path)
{
    config_ = YAML::LoadFile(path);
}

SegmentationParameters::SegmentationParameters(const std::string& path) : Parameters(path) 
{
    region_threshold = config_["region"]["threshold"].as<float>();
    region_id = config_["region"]["id"].as<float>();

    object_threshold = config_["object"]["threshold"].as<float>();
    object_id = config_["object"]["threshold"].as<float>();
} 

SegmentationParameters SegmentationParameters::Load(const std::string& path)
{
    SegmentationParameters params(path);

    return params;
}
