/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About graph processing
*/

#include <scenic/core/graphing_processor.hpp>

using namespace Scenic;

GraphingProcessor::GraphingProcessor(size_t capacity) : ThreadedProcessor<GraphingInput>(capacity)
{

}

void GraphingProcessor::setCallback(std::function<void(std::shared_ptr<Graph>)> callback)
{
    outputCallback = callback;
}

void GraphingProcessor::processBuffer()
{
    while(!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<GraphingInput> raw_input = pop(Access::PRELOCK);
            RegionGraph region_graph = RegionGraph::RegionAnalysis(*raw_input);
            ObjectGraph object_graph = ObjectGraph::ObjectAnalysis(*raw_input);
            Graph graph = region_graph + object_graph;
            //if (region_graph.isEmpty()) {
            //    std::cout << "Region Graph is Empty" << std::endl;
            //} else if (object_graph.isEmpty()) {
            //    std::cout << "Object Graph is Empty" << std::endl;
            //} else {
            //    graph = region_graph + object_graph;
            //}
            Traversability::addTraversability(graph, raw_input);
            std::shared_ptr p_graph = std::make_shared<Graph>(graph);

            if (outputCallback) outputCallback(p_graph);
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
