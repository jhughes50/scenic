/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About score the traversability of each edge
*/

#include <opencv/opencv2.hpp>

#include "scenic/graphing/graph.hpp"

struct Traversability
{
    Traversability() = default;
    static void addTraversability(Graph& graph, const GraphingInput& input);
};

void Traversability::addTraversability(Graph& graph, const GrapingInput& input)
{
    // we need to get the class of the node and its traversability score
    for (const auto [key, node] : graph.getNodes()) {
        cv::Point p1 = node->getPixelCoordinate();
        for(const std::shared_ptr<Node> connected : node.getConnections()) {
            std::shared_ptr<Edge> edge = graph(key, connected.getNodeID());
            if (edge) {
                // ensure edge exists
                cv::Point p2 = connected->getPixelCoordinate();
                cv::LineIterator it(input.logits[], p1, p2);
                float sum = 0.0;
                for (int i = 0; i < it.count; i++, ++it) {
                    sum += **it;
                }
                float avg = sum / it.count;
            }
            else {
                // todo update this
                std::cout << "Edge does not exist" << std::endl;
            }
        }
    }
}`
