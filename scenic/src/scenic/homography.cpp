#include "scenic/core/homography.hpp"
#include <opencv2/features2d.hpp>

namespace homography {

std::optional<HomographyResult> compute_homography(
    const cv::Mat& img1,
    const cv::Mat& img2,
    const HomographyParams& params)
{
    if (img1.empty() || img2.empty()) {
        return std::nullopt;
    }

    // Convert to grayscale if needed
    cv::Mat gray1, gray2;
    if (img1.channels() == 3) {
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    } else {
        gray1 = img1;
    }
    if (img2.channels() == 3) {
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    } else {
        gray2 = img2;
    }

    // Create ORB detector
    auto orb = cv::ORB::create(params.max_features);

    // Detect keypoints and compute descriptors
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desc1, desc2;
    orb->detectAndCompute(gray1, cv::noArray(), kp1, desc1);
    orb->detectAndCompute(gray2, cv::noArray(), kp2, desc2);

    if (desc1.empty() || desc2.empty()) {
        return std::nullopt;
    }

    // Match descriptors using BFMatcher with Hamming distance (for ORB)
    auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(desc1, desc2, knn_matches, 2);

    // Apply Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    for (const auto& m : knn_matches) {
        if (m.size() >= 2 && m[0].distance < params.match_ratio * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }

    // Check minimum matches
    if (static_cast<int>(good_matches.size()) < params.min_matches) {
        return std::nullopt;
    }

    // Extract matched points
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(good_matches.size());
    pts2.reserve(good_matches.size());
    for (const auto& match : good_matches) {
        pts1.push_back(kp1[match.queryIdx].pt);
        pts2.push_back(kp2[match.trainIdx].pt);
    }

    // Compute homography with RANSAC
    std::vector<char> inlier_mask;
    cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, params.ransac_threshold, inlier_mask);

    if (H.empty()) {
        return std::nullopt;
    }

    // Collect inlier matches
    std::vector<cv::DMatch> inliers;
    for (size_t i = 0; i < inlier_mask.size(); ++i) {
        if (inlier_mask[i]) {
            inliers.push_back(good_matches[i]);
        }
    }

    // Need at least 4 inliers for a valid homography
    if (inliers.size() < 4) {
        return std::nullopt;
    }

    // Compute reprojection error
    double total_error = 0.0;
    for (size_t i = 0; i < pts1.size(); ++i) {
        if (!inlier_mask[i]) continue;
        
        cv::Mat pt1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
        cv::Mat projected = H * pt1;
        projected /= projected.at<double>(2, 0);
        
        double dx = projected.at<double>(0, 0) - pts2[i].x;
        double dy = projected.at<double>(1, 0) - pts2[i].y;
        total_error += std::sqrt(dx * dx + dy * dy);
    }
    double avg_error = total_error / inliers.size();

    return HomographyResult{H, inliers, avg_error};
}

std::optional<HomographyResult> compute_homography(
    const std::string& path1,
    const std::string& path2,
    const HomographyParams& params)
{
    cv::Mat img1 = cv::imread(path1);
    cv::Mat img2 = cv::imread(path2);
    return compute_homography(img1, img2, params);
}

cv::Mat warp_image(const cv::Mat& img, const cv::Mat& H, cv::Size output_size)
{
    cv::Mat warped;
    cv::warpPerspective(img, warped, H, output_size);
    return warped;
}

} // namespace homography
