/*!
 * @Author Jason Hughes
 * @Date January 2026
 *
 * @About 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

namespace Scenic
{

struct HomographyResult 
{
    cv::Mat H;                             
    std::vector<cv::DMatch> inliers;       
    double reprojection_error;             
};

struct HomographyParams 
{
    int max_features = 1000;               
    float match_ratio = 0.75f;             
    double ransac_threshold = 3.0;         
    int min_matches = 10;                   
};

std::optional<HomographyResult> computeHomography(const cv::Mat& img1,const cv::Mat& img2,const HomographyParams& params = HomographyParams{});

cv::Mat warpImage(const cv::Mat& img, const cv::Mat& H, cv::Size output_size);

} // namespace Scenic 
