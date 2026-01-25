/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About kmeans clustering
*/

#include "scenic/graphing/kmeans.hpp"

using namespace Scenic;

KMeans::KMeans(const std::string& rect_path) 
{
    rectifier_ = Rectifier::Load(rect_path);
}

KMeansOutput KMeans::cluster(const cv::Mat mask, int k)
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

    std::vector<cv::Point> cluster_centers;
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

AdjacencyOutput KMeans::connectRegions(const std::vector<cv::Point>& points, const cv::Mat& labels, int k)
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
    AdjacencyOutput output;
    output.adjacency = adjacency_dict;
    return output; 
}

KMeansOutput KMeans::nFixedLloyds(const cv::Mat& mask, const std::vector<cv::Point2f>& fixed_centroids, int num_new_centroids, int max_iterations = 100, double tolerance = 1e-4) 
{
    std::vector<cv::Point2f> points;
    std::vector<cv::Point> ipoints;
    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) == 1) {
                points.push_back(cv::Point2f(x, y));
                ipoints.push_back(cv::Point(x, y));
            }
        }
    }

    if (points.empty() || num_new_centroids < 0) {
        throw std::invalid_argument("Invalid input parameters");
    }
    
    int num_fixed = fixed_centroids.size();
    int k = num_fixed + num_new_centroids;
    
    std::vector<cv::Point2f> centroids = fixed_centroids;
    
    std::vector<int> random_indices;
    for (int i = 0; i < points.size(); i++) {
        random_indices.push_back(i);
    }
    std::random_shuffle(random_indices.begin(), random_indices.end());
    
    for (int i = 0; i < num_new_centroids && i < points.size(); i++) {
        centroids.push_back(points[random_indices[i]]);
    }
    
    std::vector<int> labels(points.size());
    
    for (int iter = 0; iter < max_iterations; iter++) {
        
        for (size_t i = 0; i < points.size(); i++) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = 0;
            
            for (int j = 0; j < k; j++) {
                float dist = cv::norm(points[i] - centroids[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = j;
                }
            }
            
            labels[i] = best_cluster;
        }
        
        std::vector<cv::Point2f> new_centroids(k, cv::Point2f(0, 0));
        std::vector<int> cluster_sizes(k, 0);
        
        for (size_t i = 0; i < points.size(); i++) {
            int cluster = labels[i];
            new_centroids[cluster] += points[i];
            cluster_sizes[cluster]++;
        }
        
        for (int j = 0; j < k; j++) {
            if (j < num_fixed) {
                new_centroids[j] = centroids[j];
            } else {
                if (cluster_sizes[j] > 0) {
                    new_centroids[j] *= (1.0f / cluster_sizes[j]);
                } else {
                    new_centroids[j] = points[rand() % points.size()];
                }
            }
        }
        
        double max_shift = 0.0;
        for (int j = num_fixed; j < k; j++) {
            double shift = cv::norm(new_centroids[j] - centroids[j]);
            max_shift = std::max(max_shift, shift);
        }
        
        centroids = new_centroids;
        
        if (max_shift < tolerance) {
            //std::cout << "Converged after " << iter + 1 << " iterations" << std::endl;
            break;
        }
    }
    
    double inertia = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
        double dist = cv::norm(points[i] - centroids[labels[i]]);
        inertia += dist * dist;
    }

    std::vector<cv::Point> new_centers;
    for(size_t i = fixed_centroids.size(); i < centroids.size(); i++) {
        cv::Point p(centroids[i]);
        new_centers.push_back(p);
    }

    cv::Mat voronoi = cv::Mat::zeros(mask.size(), CV_8U);
    for (int i = 0; i < points.size(); i++) {
        int cluster = labels[i] + 1;
        cv::Point pt(points[i]);
        voronoi.at<uchar>(pt) = cluster;
    }

    return {new_centers, voronoi, ipoints};
}

int KMeans::getNumClusters(const cv::Mat& mask, int alt)
{
    double fov = rectifier_.getHorizontalFov();
    double angle = 180.0 - (90.0+fov/2.0);
    double gs = 2 * (alt / std::tan(angle * M_PI / 180.0));
    double gs_pp = gs / mask.cols;
    double pixel_area = gs_pp * gs_pp;

    cv::Scalar temp = cv::sum(mask);
    double mask_area_pixels = temp[0];
    double mask_area_m = mask_area_pixels * pixel_area;

    int num_regions = static_cast<int>(std::floor(mask_area_m / region_area_));
    return std::min(max_regions_, num_regions);
}
