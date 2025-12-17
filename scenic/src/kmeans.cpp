/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About kmeans clustering
*/

#include "scenic/graphing/kmeans.hpp"

using namespace Scenic;

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


    cv::Mat voronoi = cv::Mat::zeros(mask.size(), CV_8U);
    for (int i = 0; i < points.size(); i++) {
        int cluster = labels.at<int>(i);
        voronoi.at<uchar>(points[i]) = cluster * (255 / (k - 1));  // Map 0,1,2 to 0,127,255
    }

    std::vector<Point> cluster_centers;
    for (int i = 0; i < k; i++) {
        cluster_centers.push_back({
            static_cast<int>(centers.at<float>(i, 0)),
            static_cast<int>(centers.at<float>(i, 1))
        });
    }


    // TODO post process
    KMeansOutput output;
    output.voronoi = voronoi;
    output.centroids = cluster_centers;

    return output;
}

std::unordered_map<int, std::unordered_set<int>> KMeans::findAdjacentRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k)
{
    std::unordered_map<int, std::unordered_set<int>> adjacency_dict;
    std::unordered_map<int, Point> centroids;
    
    // Build spatial lookup - map from pixel location to cluster
    std::unordered_map<int, int> pixel_to_cluster;  // key: y*width + x, value: cluster_id
    int width = 1000;  // Set to your image width
    
    for (int i = 0; i < points.size(); i++) {
        int cluster = labels.at<int>(i);
        int key = points[i].y * width + points[i].x;
        pixel_to_cluster[key] = cluster;
    }
    
    // For each point, check its 8 neighbors
    for (int i = 0; i < points.size(); i++) {
        int cluster = labels.at<int>(i);
        int x = points[i].x;
        int y = points[i].y;
        
        // Check 8-connected neighbors
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                
                int nx = x + dx;
                int ny = y + dy;
                int neighbor_key = ny * width + nx;
                
                auto it = pixel_to_cluster.find(neighbor_key);
                if (it != pixel_to_cluster.end()) {
                    int neighbor_cluster = it->second;
                    if (neighbor_cluster != cluster) {
                        adjacency_dict[cluster].insert(neighbor_cluster);
                    }
                }
            }
        }
    }
    
    // Build centroids dictionary (1-indexed)
    for (int i = 0; i < k; i++) {
        centroids[i + 1] = centers[i];
    }
    
    return {adjacency_dict, centroids}; 
}
