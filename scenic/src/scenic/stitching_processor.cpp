/*!
* @Author Jason Hughes
* @Date January 2025
*
* @About stitch the graphs together
*/

#include "scenic/core/stitching_processor.cpp"

StitchingProcessor::StitchingProcessor(size_t capacity, const std::string rect_path) : ThreadedProcessor(capacity)
{
    rectifier_ = Rectifier::Load(rect_path, 4);
}

void StitchingProcessor::setCallback(std::function<void(std::shared_ptr<Graph>)> callback)
{
    outputCallback = callback;
}

void StitchingProcessor::localizeNodes(std::shared_ptr<Graph> graph)
{
    // TODO;
}

void StitchingProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            if (!scene_graph_) {
                // graph is not initialized so we do that
                // this should only happend the first entry,
                // or if we need to reset for some reason
                scene_graph_ = std::move(pop(Access::PRELOCK));
                //TODO localize points
            } else {
                // TODO localize points in new graph and see if there in scene_graph
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (scenc_graph_) outputCallback(scene_graph_);
    }
}
