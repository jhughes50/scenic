/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About graph processing
*/

#include <scenic/core/graphing_processor.hpp>

using namespace Scenic;

GraphingProcessor::GraphingProcessor(size_t capacity, const std::string& rect_path) : ThreadedProcessor<GraphingInput>(capacity), kmeans_(rect_path)
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
            RegionGraph region_graph = RegionGraph::RegionAnalysis(*raw_input, kmeans_);
            ObjectGraph object_graph = ObjectGraph::ObjectAnalysis(*raw_input);
            Graph graph = region_graph + object_graph;
            int pid = raw_input->pid;
            graph.setProcessID(pid);
            if (region_graph.isEmpty()) {
                LOG(WARNING) << "[SCENIC] Region Graph is Empty at PID" << pid;
                continue;
            } else if (object_graph.isEmpty()) {
                LOG(WARNING) << "[SCENIC] Object Graph is Empty at PID" << pid;
            } 
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
