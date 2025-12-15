/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About kmeans clustering
*/

#include "scenic/core/kmeans.hpp"

using namespace scenic;

KMeansOutput KMeans::operator()(const cv::Mat mask, int k)
{
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);

    cv::Mat data(points.size(), 2, CV_32F);
    for (int i = 0; i < points.size(); i++) {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
    }

    cv::Mat labels, centers;
    cv::kmeans(data, 
               k, 
               labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1),
               3,
               cv::KMEANS_PP_CENTERS, centers);

    // TODO post process
    KMeansOutput output;
    return output;
}
