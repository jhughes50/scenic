/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About graph scructure
*/

#include "scenic/graphing/graph.hpp"

using namespace Scenic;

Graph Graph::SingleImageAnalysis(const GraphingInput& input)
{
    size_t input_size = input.getSize();

    cv::Mat region_mask = cv::Mat::zeros(input.image.rows, input.image.cols, CV_8UC1);
    for (size_t i = 0; i < inputs_size; ++i) {
        if (input.texts[i].level == 0) {
            cv::bitwise(region_mask, input.masks[i], region_mask);
        }
    }

    //int k = getNumClusters(region_mask);
    int k = 3;
    KMeansOutput output = KMeans::Cluster(region_mask, k); 
    std::unordered_map<int, std::unordered_set<int>> graph = KMeans::findAdjacentRegions(output.points, output.voronoi, k);


}

