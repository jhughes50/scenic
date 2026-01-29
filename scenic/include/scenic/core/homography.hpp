#ifndef HOMOGRAPHY_HPP
#define HOMOGRAPHY_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

namespace homography {

struct HomographyResult {
    cv::Mat H;                              // 3x3 homography matrix
    std::vector<cv::DMatch> inliers;        // inlier matches used
    double reprojection_error;              // average reprojection error
};

struct HomographyParams {
    int max_features = 1000;                // max ORB features to detect
    float match_ratio = 0.75f;              // Lowe's ratio test threshold
    double ransac_threshold = 3.0;          // RANSAC reprojection threshold
    int min_matches = 10;                   // minimum matches required
};

/**
 * Compute homography between two images using ORB features.
 * 
 * @param img1 First image (source)
 * @param img2 Second image (destination)
 * @param params Optional parameters for feature detection and matching
 * @return HomographyResult if successful, std::nullopt if homography cannot be computed
 */
std::optional<HomographyResult> compute_homography(
    const cv::Mat& img1,
    const cv::Mat& img2,
    const HomographyParams& params = HomographyParams{}
);

/**
 * Compute homography from file paths.
 */
std::optional<HomographyResult> compute_homography(
    const std::string& path1,
    const std::string& path2,
    const HomographyParams& params = HomographyParams{}
);

/**
 * Warp img1 to img2's perspective using the computed homography.
 */
cv::Mat warp_image(const cv::Mat& img, const cv::Mat& H, cv::Size output_size);

} // namespace homography

#endif // HOMOGRAPHY_HPP
