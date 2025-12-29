/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About score the traversability of each edge
*/

#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>

#include "scenic/core/processor_inputs.hpp"
#include "scenic/graphing/graph.hpp"

namespace Scenic
{
struct Traversability
{
    Traversability() = default;
    static void addTraversability(Graph& graph, const std::unique_ptr<GraphingInput>& input);
};

inline void Traversability::addTraversability(Graph& graph, const std::unique_ptr<GraphingInput>& input)
{
    // we need to get the class of the node and its traversability score
    for (const auto [key, node] : graph.getRegionNodes()) {
        cv::Point p1 = node->getPixelCoordinate();
        int node_lbl = node->getClassLabel();
        for(const std::shared_ptr<Node> connected : node->getConnectedNodes()) {
            uint64_t cid = connected->getNodeID();
            std::shared_ptr<Edge> edge = graph(key, cid);
            if (edge) {
                // ensure edge exists
                cv::Point p2 = connected->getPixelCoordinate();
                cv::LineIterator it(input->image, p1, p2);
                int conn_lbl = connected->getClassLabel();
                
                // get the correct multiplier and logits
                float multiplier;
                cv::Mat logits, conn_logits;
                if (node_lbl == conn_lbl) {
                    // node labels are the same
                    multiplier = input->getMultiplierFromClassLabel(node_lbl);
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                } else if (connected->getNodeLevel() != GraphLevel::OBJECT) {
                    // node labels are diff use the min 
                    float node_mul = input->getMultiplierFromClassLabel(node_lbl);
                    float conn_mul = input->getMultiplierFromClassLabel(conn_lbl);
                    multiplier = std::fmin(node_mul, conn_mul);
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                    conn_logits = input->getNormalizedLogitsFromClassLabel(conn_lbl);
                } else {
                    multiplier = input->getMultiplierFromClassLabel(node_lbl); 
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                }

                float sum = 0.0;
                for (int i = 0; i < it.count; i++, ++it) {
                    cv::Point pos = it.pos();

                    if (conn_logits.empty()) {
                        float l = logits.at<float>(pos);
                        sum += l;
                    } else {
                        float l = logits.at<float>(pos);
                        float c = conn_logits.at<float>(pos);
                        float ml = std::fmax(l,c);
                        sum += ml;
                    }
                }
                float avg = sum/static_cast<float>(it.count);
                float score = multiplier * avg;
                edge->setScore(score);
            }
            else {
                // todo update this
                std::cerr << "[SCENIC] Attempting to score an edge that does not exist" << std::endl;
            }
        }
    }
}
}
