/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About score the traversability of each edge
*/

#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
            
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
    for (const std::shared_ptr<Node>& node : graph.getRegionNodes()) {
        uint64_t key = node->getNodeID();
        cv::Point p1 = node->getPixelCoordinate();
        size_t node_lbl = node->getClassLabel();
        for(const std::shared_ptr<Node> connected : node->getConnectedNodes()) {
            uint64_t cid = connected->getNodeID();
            std::shared_ptr<Edge> edge = graph(key, cid);
            if (edge) {
                // ensure edge exists
                cv::Point p2 = connected->getPixelCoordinate();
                cv::LineIterator it(input->image, p1, p2);
                size_t conn_lbl = connected->getClassLabel();
                
                // get the correct multiplier and logits
                float multiplier;
                cv::Mat logits, conn_logits, mask, conn_mask;
                if (node_lbl == conn_lbl) {
                    // node labels are the same
                    multiplier = input->getMultiplierFromClassLabel(node_lbl);
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                    mask = input->getMaskFromClassLabel(node_lbl);
                } else if (connected->getNodeLevel() != GraphLevel::OBJECT) {
                    // node labels are diff use the min 
                    float node_mul = input->getMultiplierFromClassLabel(node_lbl);
                    float conn_mul = input->getMultiplierFromClassLabel(conn_lbl);
                    multiplier = std::fmin(node_mul, conn_mul);
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                    conn_logits = input->getNormalizedLogitsFromClassLabel(conn_lbl);
                    mask = input->getMaskFromClassLabel(node_lbl);
                    conn_mask = input->getMaskFromClassLabel(conn_lbl);
                } else {
                    // edge is to an object
                    multiplier = input->getMultiplierFromClassLabel(node_lbl); 
                    logits = input->getNormalizedLogitsFromClassLabel(node_lbl);
                    // unsafe connection to object
                    mask = cv::Mat::ones(logits.size(), CV_8U);
                }

                float sum = 0.0;
                int mask_sum = 0;
                bool warned = false;
                for (int i = 0; i < it.count; i++, ++it) {
                    cv::Point pos = it.pos();

                    if (pos.y < 0 || pos.y >= logits.rows || pos.x < 0 || pos.x >= logits.cols) {
                        if (!warned) {
                            LOG(WARNING) << "[SCENIC] Node Pixel Coord Out Of Bounds";
                            warned = true;
                        }
                        continue; 
                    }

                    if (conn_logits.empty()) {
                        float l = logits.at<float>(pos);
                        uint8_t m = mask.at<uint8_t>(pos); 
                        sum += l;
                        mask_sum += static_cast<int>(m);
                    } else {
                        float l = logits.at<float>(pos);
                        float c = conn_logits.at<float>(pos);
                        uint8_t m = mask.at<uint8_t>(pos);
                        uint8_t mc = mask.at<uint8_t>(pos);
                        float ml = std::fmax(l,c);
                        uint8_t mm = std::max<uint8_t>(m, mc);
                        sum += ml;
                        mask_sum += static_cast<int>(mm);
                    }
                }
                float avg = sum/static_cast<float>(it.count);
                float pct = static_cast<float>(mask_sum)/static_cast<float>(it.count);
                if (pct > 0.95) {
                    float score = multiplier * avg;
                    edge->setScore(score);
                } else {
                    graph.pruneEdge(edge); 
                }
            }
            else {
                // todo update this
                LOG(WARNING) << "[SCENIC] Attempting to score an edge that does not exist";
            } 
        }
    }
}
}
