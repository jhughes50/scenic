/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About kmeans clustering
*/

#include "scenic/graphing/kmeans.hpp"

using namespace Scenic;

KMeansOutput KMeans::Cluster(const cv::Mat mask, int k)
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


    cv::Mat voronoi = cv::Mat::zeros(mask.size(), CV_8U);
    for (int i = 0; i < points.size(); i++) {
        int cluster = labels.at<int>(i)+1;
        voronoi.at<uchar>(points[i]) = cluster;  // Map 0,1,2 to 0,127,255
    }

    std::vector<Point> cluster_centers;
    for (int i = 0; i < k; i++) {
        cluster_centers.push_back({
            static_cast<int>(centers.at<float>(i, 0)),
            static_cast<int>(centers.at<float>(i, 1))
        });
    }

    KMeansOutput output;
    output.voronoi = voronoi;
    output.centroids = cluster_centers;
    output.points = points;

    return output;
}

std::unordered_map<uchar, std::unordered_set<uchar>> KMeans::findAdjacentRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k)
{
    std::unordered_map<uchar, std::unordered_set<uchar>> adjacency_dict;
    
    for (int i = 0; i < points.size(); i++) {
        int x = points[i].x;
        int y = points[i].y;
        uchar cluster = labels.at<uchar>(y,x);

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < labels.cols && ny >= 0 && ny < labels.rows) {
                    uchar neighbor_cluster = labels.at<uchar>(ny, nx);
                    if (neighbor_cluster != 0 && cluster != neighbor_cluster) {
                        adjacency_dict[cluster].insert(neighbor_cluster);
                    }
                }

            }
        }
    }
    
    return adjacency_dict; 
}
