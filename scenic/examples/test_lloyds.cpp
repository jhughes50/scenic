#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>
#include <numeric>

struct KMeansResult {
    std::vector<cv::Point2f> centroids;
    std::vector<int> labels;
    double inertia;
};

KMeansResult lloydsAlgorithmWithFixedCentroids(
    const std::vector<cv::Point2f>& points,
    const std::vector<cv::Point2f>& fixedCentroids,
    int numNewCentroids,
    int maxIterations = 100,
    double tolerance = 1e-4) {
    
    if (points.empty() || numNewCentroids < 0) {
        throw std::invalid_argument("Invalid input parameters");
    }
    
    int numFixed = fixedCentroids.size();
    int k = numFixed + numNewCentroids;
    
    // Step 1: Initialize centroids
    // First add the fixed centroids
    std::vector<cv::Point2f> centroids = fixedCentroids;
    
    // Then initialize new centroids randomly
    std::vector<int> randomIndices;
    for (int i = 0; i < points.size(); i++) {
        randomIndices.push_back(i);
    }
    std::random_shuffle(randomIndices.begin(), randomIndices.end());
    
    for (int i = 0; i < numNewCentroids && i < points.size(); i++) {
        centroids.push_back(points[randomIndices[i]]);
    }
    
    std::vector<int> labels(points.size());
    
    // Lloyd's algorithm iterations
    for (int iter = 0; iter < maxIterations; iter++) {
        
        // Step 2: Assignment step - assign each point to nearest centroid
        for (size_t i = 0; i < points.size(); i++) {
            float minDist = std::numeric_limits<float>::max();
            int bestCluster = 0;
            
            for (int j = 0; j < k; j++) {
                float dist = cv::norm(points[i] - centroids[j]);
                if (dist < minDist) {
                    minDist = dist;
                    bestCluster = j;
                }
            }
            
            labels[i] = bestCluster;
        }
        
        // Step 3: Update step - recompute centroids
        std::vector<cv::Point2f> newCentroids(k, cv::Point2f(0, 0));
        std::vector<int> clusterSizes(k, 0);
        
        // Sum up all points in each cluster
        for (size_t i = 0; i < points.size(); i++) {
            int cluster = labels[i];
            newCentroids[cluster] += points[i];
            clusterSizes[cluster]++;
        }
        
        // Compute mean for each cluster
        for (int j = 0; j < k; j++) {
            if (j < numFixed) {
                // Keep fixed centroids unchanged
                newCentroids[j] = centroids[j];
            } else {
                // Update new centroids
                if (clusterSizes[j] > 0) {
                    newCentroids[j] *= (1.0f / clusterSizes[j]);
                } else {
                    // Handle empty cluster - reinitialize randomly
                    newCentroids[j] = points[rand() % points.size()];
                }
            }
        }
        
        // Step 4: Check for convergence (only check new centroids)
        double maxShift = 0.0;
        for (int j = numFixed; j < k; j++) {
            double shift = cv::norm(newCentroids[j] - centroids[j]);
            maxShift = std::max(maxShift, shift);
        }
        
        centroids = newCentroids;
        
        if (maxShift < tolerance) {
            std::cout << "Converged after " << iter + 1 << " iterations" << std::endl;
            break;
        }
    }
    
    // Calculate final inertia
    double inertia = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
        double dist = cv::norm(points[i] - centroids[labels[i]]);
        inertia += dist * dist;
    }
    
    return {centroids, labels, inertia};
}

int main() {
    // Load binary mask
    cv::Mat mask = cv::imread("../test/test_mask_road.png", cv::IMREAD_GRAYSCALE);
    
    if (mask.empty()) {
        std::cerr << "Error: Could not load mask image" << std::endl;
        return -1;
    }
    
    // Convert mask to binary (0 or 1) if needed
    cv::threshold(mask, mask, 127, 1, cv::THRESH_BINARY);
    
    // Extract foreground pixels (value=1) as points
    auto startExtract = std::chrono::high_resolution_clock::now();
    
    std::vector<cv::Point2f> points;
    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) == 1) {
                points.push_back(cv::Point2f(x, y));
            }
        }
    }
    
    auto endExtract = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> extractTime = endExtract - startExtract;
    
    std::cout << "Found " << points.size() << " foreground pixels" << std::endl;
    std::cout << "Extraction time: " << extractTime.count() << " ms" << std::endl;
    
    if (points.empty()) {
        std::cerr << "Error: No foreground pixels found in mask" << std::endl;
        return -1;
    }
    
    // Define fixed centroids
    std::vector<cv::Point2f> fixedCentroids = {
        cv::Point2f(314.797, 189.693),
        cv::Point2f(245.182, 275.348)
    };
    
    // Run Lloyd's algorithm with 2 fixed centroids and 2 new centroids
    int numNewCentroids = 2;
    
    auto startKmeans = std::chrono::high_resolution_clock::now();
    KMeansResult result = lloydsAlgorithmWithFixedCentroids(points, fixedCentroids, numNewCentroids);
    auto endKmeans = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double, std::milli> kmeansTime = endKmeans - startKmeans;
    
    std::cout << "\nFixed centroids:" << std::endl;
    for (size_t i = 0; i < fixedCentroids.size(); i++) {
        std::cout << "  Cluster " << i << " (FIXED): (" 
                  << result.centroids[i].x << ", " 
                  << result.centroids[i].y << ")" << std::endl;
    }
    
    std::cout << "\nNew centroids:" << std::endl;
    for (size_t i = fixedCentroids.size(); i < result.centroids.size(); i++) {
        std::cout << "  Cluster " << i << " (NEW): (" 
                  << result.centroids[i].x << ", " 
                  << result.centroids[i].y << ")" << std::endl;
    }
    
    std::cout << "\nInertia: " << result.inertia << std::endl;
    std::cout << "K-means compute time: " << kmeansTime.count() << " ms" << std::endl;
    
    auto totalTime = extractTime + kmeansTime;
    std::cout << "Total compute time: " << totalTime.count() << " ms" << std::endl;
    
    // Visualize results - create RGB image from mask
    cv::Mat image;
    cv::cvtColor(mask * 255, image, cv::COLOR_GRAY2BGR);
    
    cv::Scalar colors[] = {cv::Scalar(255, 0, 0),   // Blue - Fixed
                          cv::Scalar(0, 255, 0),     // Green - Fixed
                          cv::Scalar(0, 0, 255),     // Red - New
                          cv::Scalar(255, 255, 0)};  // Cyan - New
    
    // Draw clustered points
    for (size_t i = 0; i < points.size(); i++) {
        cv::circle(image, points[i], 1, colors[result.labels[i]], -1);
    }
    
    // Draw centroids
    for (size_t i = 0; i < result.centroids.size(); i++) {
        if (i < fixedCentroids.size()) {
            // Draw fixed centroids with a square
            cv::rectangle(image, 
                         cv::Point(result.centroids[i].x - 8, result.centroids[i].y - 8),
                         cv::Point(result.centroids[i].x + 8, result.centroids[i].y + 8),
                         cv::Scalar(0, 0, 0), 2);
            cv::rectangle(image, 
                         cv::Point(result.centroids[i].x - 6, result.centroids[i].y - 6),
                         cv::Point(result.centroids[i].x + 6, result.centroids[i].y + 6),
                         colors[i], -1);
        } else {
            // Draw new centroids with a circle
            cv::circle(image, result.centroids[i], 10, cv::Scalar(0, 0, 0), 2);
            cv::circle(image, result.centroids[i], 8, colors[i], -1);
        }
    }
    
    cv::imshow("K-means Clustering on Mask", image);
    cv::waitKey(0);
    
    return 0;
}
